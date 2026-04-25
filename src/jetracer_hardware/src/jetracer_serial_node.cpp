#include <boost/asio.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr uint8_t kHead1 = 0xAA;
constexpr uint8_t kHead2 = 0x55;
constexpr uint8_t kSendTypeVelocity = 0x11;
constexpr uint8_t kSendTypeParams = 0x12;
constexpr uint8_t kSendTypeCoefficient = 0x13;
constexpr uint8_t kRecvTypeTelemetry = 0x20;  // RP2040 telemetry response
constexpr std::size_t kMaxFrameSize = 64;

// RP2040 telemetry structure (extended from firmware)
struct RP2040Telemetry {
  uint16_t firmware_version;      // CRC of firmware blob
  uint8_t watchdog_resets_count;  // Count since Jetson boot
  uint8_t estop_reason;           // Why E-stop was triggered
  uint8_t command_ack_rate{100};  // % of commands acknowledged
  int16_t steering_angle_measured; // Potentiometer feedback (optional)
  uint16_t voltage_rail_3v3;      // 3.3V rail voltage (mV)
  uint16_t voltage_rail_5v0;      // 5.0V rail voltage (mV) if available
  uint8_t reserved;
};

uint8_t checksum(const uint8_t * buffer, std::size_t length)
{
  uint8_t sum = 0;
  for (std::size_t index = 0; index < length; ++index) {
    sum = static_cast<uint8_t>(sum + buffer[index]);
  }
  return sum;
}

int16_t decode_int16(uint8_t high, uint8_t low)
{
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
}
}  // namespace

class JetRacerSerialNode : public rclcpp::Node
{
public:
  JetRacerSerialNode()
  : Node("jetracer_hardware"),
    serial_port_(io_service_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<bool>("publish_odom_transform", false);
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "base_footprint");
    declare_parameter<std::string>("imu_frame_id", "imu_link");
    declare_parameter<double>("linear_correction", 1.0);
    declare_parameter<double>("coefficient_a", -0.016073);
    declare_parameter<double>("coefficient_b", 0.176183);
    declare_parameter<double>("coefficient_c", -23.428084);
    declare_parameter<double>("coefficient_d", 1500.0);
    declare_parameter<int>("kp", 350);
    declare_parameter<int>("ki", 120);
    declare_parameter<int>("kd", 0);
    declare_parameter<int>("servo_bias", 0);
    declare_parameter<double>("command_timeout_sec", 1.0);
    declare_parameter<double>("send_period_sec", 0.02);
    declare_parameter<double>("wheel_radius", 0.035);
    declare_parameter<double>("max_linear_velocity", 1.2);
    declare_parameter<double>("max_lateral_velocity", 0.0);
    declare_parameter<double>("max_steering_angle", 0.6);
    // Safety additions
    declare_parameter<bool>("dry_run", false);
    declare_parameter<double>("max_accel_mps2", 0.5);
    declare_parameter<double>("startup_speed_limit_ms", 0.20);
    declare_parameter<double>("startup_duration_sec", 30.0);

    load_parameters();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_raw", 10);
    left_velocity_pub_ = create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
    right_velocity_pub_ = create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
    left_setpoint_pub_ = create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
    right_setpoint_pub_ = create_publisher<std_msgs::msg::Int32>("motor/rset", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&JetRacerSerialNode::cmd_callback, this, std::placeholders::_1));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&JetRacerSerialNode::parameter_callback, this, std::placeholders::_1));

    last_receive_time_ = now();
    last_command_time_ = now();
    node_start_time_ = now();

    if (dry_run_) {
      RCLCPP_WARN(get_logger(),
        "[DRY-RUN] Hardware serial writes are SUPPRESSED. "
        "Serial port will not be opened and commands will NOT be sent to motors.");
    } else {
      open_serial_port();
      send_coefficient();
      send_params();
      serial_thread_ = std::thread(&JetRacerSerialNode::serial_task, this);
    }
    if (startup_duration_sec_ > 0.0) {
      RCLCPP_INFO(get_logger(),
        "Startup speed limit active: %.2f m/s for %.0f s.",
        startup_speed_limit_ms_, startup_duration_sec_);
    }

    const auto send_period = std::chrono::duration<double>(send_period_sec_);
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(send_period),
      std::bind(&JetRacerSerialNode::send_velocity_timer, this));

    // Initialize Diagnostic Updater
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("Waveshare_JetRacer_RP2040");
    diagnostic_updater_->add("Serial Connection", this, &JetRacerSerialNode::diagnostic_serial);
    diagnostic_updater_->add("Command Heartbeat", this, &JetRacerSerialNode::diagnostic_heartbeat);
    diagnostic_updater_->add("Sensor Stream", this, &JetRacerSerialNode::diagnostic_sensors);
    diagnostic_updater_->add("Battery Health", this, &JetRacerSerialNode::diagnostic_battery);
    diagnostic_updater_->add("RP2040 Status", this, &JetRacerSerialNode::diagnostic_rp2040);
  }

  ~JetRacerSerialNode() override
  {
    stop_requested_.store(true);

    // EMERGENCY STOP: Send zero-velocity frame before port closure
    if (serial_port_.is_open()) {
      send_velocity_frame(0.0, 0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow serial drain
      
      boost::system::error_code error_code;
      serial_port_.cancel(error_code);
      serial_port_.close(error_code);
    }
    
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }
  }

private:
  void load_parameters()
  {
    port_name_ = get_parameter("port_name").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    publish_odom_transform_ = get_parameter("publish_odom_transform").as_bool();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    imu_frame_id_ = get_parameter("imu_frame_id").as_string();
    linear_correction_ = get_parameter("linear_correction").as_double();
    coefficient_a_ = get_parameter("coefficient_a").as_double();
    coefficient_b_ = get_parameter("coefficient_b").as_double();
    coefficient_c_ = get_parameter("coefficient_c").as_double();
    coefficient_d_ = get_parameter("coefficient_d").as_double();
    kp_ = get_parameter("kp").as_int();
    ki_ = get_parameter("ki").as_int();
    kd_ = get_parameter("kd").as_int();
    servo_bias_ = get_parameter("servo_bias").as_int();
    command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
    send_period_sec_ = get_parameter("send_period_sec").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    max_linear_velocity_ = std::abs(get_parameter("max_linear_velocity").as_double());
    max_lateral_velocity_ = std::abs(get_parameter("max_lateral_velocity").as_double());
    max_steering_angle_ = std::abs(get_parameter("max_steering_angle").as_double());
    dry_run_ = get_parameter("dry_run").as_bool();
    max_accel_mps2_ = std::abs(get_parameter("max_accel_mps2").as_double());
    startup_speed_limit_ms_ = std::abs(get_parameter("startup_speed_limit_ms").as_double());
    startup_duration_sec_ = std::abs(get_parameter("startup_duration_sec").as_double());
  }

  void open_serial_port()
  {
    boost::system::error_code error_code;
    serial_port_.open(port_name_, error_code);
    if (error_code) {
      throw std::runtime_error("Failed to open serial port " + port_name_ + ": " + error_code.message());
    }

    serial_port_.set_option(boost::asio::serial_port::baud_rate(baud_rate_));
    serial_port_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_port_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_port_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port::character_size(8));

    RCLCPP_INFO(get_logger(), "Opened serial port %s at %d baud.", port_name_.c_str(), baud_rate_);
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::scoped_lock lock(command_mutex_);
    commanded_x_ = msg->linear.x;
    commanded_y_ = msg->linear.y;
    commanded_yaw_ = msg->angular.z;
    last_command_time_ = now();
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "publish_odom_transform") {
        publish_odom_transform_ = parameter.as_bool();
      } else if (name == "odom_frame_id") {
        odom_frame_id_ = parameter.as_string();
      } else if (name == "base_frame_id") {
        base_frame_id_ = parameter.as_string();
      } else if (name == "imu_frame_id") {
        imu_frame_id_ = parameter.as_string();
      } else if (name == "linear_correction") {
        linear_correction_ = parameter.as_double();
      } else if (name == "coefficient_a") {
        coefficient_a_ = parameter.as_double();
      } else if (name == "coefficient_b") {
        coefficient_b_ = parameter.as_double();
      } else if (name == "coefficient_c") {
        coefficient_c_ = parameter.as_double();
      } else if (name == "coefficient_d") {
        coefficient_d_ = parameter.as_double();
      } else if (name == "kp") {
        kp_ = parameter.as_int();
      } else if (name == "ki") {
        ki_ = parameter.as_int();
      } else if (name == "kd") {
        kd_ = parameter.as_int();
      } else if (name == "servo_bias") {
        servo_bias_ = parameter.as_int();
      } else if (name == "command_timeout_sec") {
        command_timeout_sec_ = parameter.as_double();
      } else if (name == "send_period_sec") {
        send_period_sec_ = parameter.as_double();
        // Dynamic timer reset
        const auto send_period = std::chrono::duration<double>(send_period_sec_);
        send_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(send_period),
          std::bind(&JetRacerSerialNode::send_velocity_timer, this));
      } else if (name == "wheel_radius") {
        wheel_radius_ = parameter.as_double();
      } else if (name == "max_linear_velocity") {
        max_linear_velocity_ = std::abs(parameter.as_double());
      } else if (name == "max_lateral_velocity") {
        max_lateral_velocity_ = std::abs(parameter.as_double());
      } else if (name == "max_steering_angle") {
        max_steering_angle_ = std::abs(parameter.as_double());
      } else if (name == "dry_run") {
        dry_run_ = parameter.as_bool();
        RCLCPP_WARN(get_logger(), "dry_run set to %s at runtime.", dry_run_ ? "TRUE" : "FALSE");
      } else if (name == "max_accel_mps2") {
        max_accel_mps2_ = std::abs(parameter.as_double());
      } else if (name == "startup_speed_limit_ms") {
        startup_speed_limit_ms_ = std::abs(parameter.as_double());
      } else if (name == "startup_duration_sec") {
        startup_duration_sec_ = std::abs(parameter.as_double());
      }
    }

    send_params();
    send_coefficient();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void encode_float(double value, uint8_t * destination)
  {
    const float float_value = static_cast<float>(value);
    std::memcpy(destination, &float_value, sizeof(float_value));
  }

  void send_params()
  {
    std::array<uint8_t, 15> buffer{};
    buffer[0] = kHead1;
    buffer[1] = kHead2;
    buffer[2] = 0x0F;
    buffer[3] = kSendTypeParams;
    buffer[4] = static_cast<uint8_t>((kp_ >> 8) & 0xFF);
    buffer[5] = static_cast<uint8_t>(kp_ & 0xFF);
    buffer[6] = static_cast<uint8_t>((ki_ >> 8) & 0xFF);
    buffer[7] = static_cast<uint8_t>(ki_ & 0xFF);
    buffer[8] = static_cast<uint8_t>((kd_ >> 8) & 0xFF);
    buffer[9] = static_cast<uint8_t>(kd_ & 0xFF);
    const auto linear_int = static_cast<int16_t>(linear_correction_ * 1000.0);
    buffer[10] = static_cast<uint8_t>((linear_int >> 8) & 0xFF);
    buffer[11] = static_cast<uint8_t>(linear_int & 0xFF);
    const auto servo_int = static_cast<int16_t>(servo_bias_);
    buffer[12] = static_cast<uint8_t>((servo_int >> 8) & 0xFF);
    buffer[13] = static_cast<uint8_t>(servo_int & 0xFF);
    buffer[14] = checksum(buffer.data(), buffer.size() - 1U);
    write_bytes(buffer.data(), buffer.size());
  }

  void send_coefficient()
  {
    std::array<uint8_t, 21> buffer{};
    buffer[0] = kHead1;
    buffer[1] = kHead2;
    buffer[2] = 0x15;
    buffer[3] = kSendTypeCoefficient;
    encode_float(coefficient_a_, buffer.data() + 4);
    encode_float(coefficient_b_, buffer.data() + 8);
    encode_float(coefficient_c_, buffer.data() + 12);
    encode_float(coefficient_d_, buffer.data() + 16);
    buffer[20] = checksum(buffer.data(), buffer.size() - 1U);
    write_bytes(buffer.data(), buffer.size());
  }

  void send_velocity_timer()
  {
    double command_x = 0.0;
    double command_y = 0.0;
    double command_yaw = 0.0;
    rclcpp::Time command_stamp(0, 0, get_clock()->get_clock_type());

    // Update Diagnostics
    if (diagnostic_updater_) {
      diagnostic_updater_->update();
    }

    {
      std::scoped_lock lock(command_mutex_);
      command_x = commanded_x_;
      command_y = commanded_y_;
      command_yaw = commanded_yaw_;
      command_stamp = last_command_time_;
    }

    // Watchdog: zero velocity if command stream is stale
    if ((now() - command_stamp).seconds() > command_timeout_sec_) {
      command_x = 0.0;
      command_y = 0.0;
      command_yaw = 0.0;
    }

    // Startup speed cap: apply conservative limit during bring-up window
    if (startup_duration_sec_ > 0.0) {
      const double elapsed = (now() - node_start_time_).seconds();
      if (elapsed < startup_duration_sec_) {
        const double lim = startup_speed_limit_ms_;
        command_x = std::clamp(command_x, -lim, lim);
      }
    }

    // Acceleration ramp: prevent velocity jumps from watchdog re-enable or
    // sudden large commands. Applied on the outgoing command before clamping.
    if (max_accel_mps2_ > 0.0 && send_period_sec_ > 0.0) {
      const double max_delta = max_accel_mps2_ * send_period_sec_;
      const double diff = command_x - prev_command_x_;
      if (std::abs(diff) > max_delta) {
        command_x = prev_command_x_ + std::copysign(max_delta, diff);
      }
    }
    prev_command_x_ = command_x;

    send_velocity_frame(command_x, command_y, command_yaw);
  }

  void send_velocity_frame(double x, double y, double yaw)
  {
    std::array<uint8_t, 11> buffer{};
    buffer[0] = kHead1;
    buffer[1] = kHead2;
    buffer[2] = 0x0B;
    buffer[3] = kSendTypeVelocity;

    const double linear_limit = std::abs(max_linear_velocity_);
    const double lateral_limit = std::abs(max_lateral_velocity_);
    const double steering_limit = std::abs(max_steering_angle_);
    const double clamped_x = std::clamp(x, -linear_limit, linear_limit);
    const double clamped_y = std::clamp(y, -lateral_limit, lateral_limit);
    const double clamped_yaw = std::clamp(yaw, -steering_limit, steering_limit);

    const auto linear_x = static_cast<int16_t>(clamped_x * 1000.0);
    const auto linear_y = static_cast<int16_t>(clamped_y * 1000.0);
    const auto angular_z = static_cast<int16_t>(clamped_yaw * 1000.0);

    buffer[4] = static_cast<uint8_t>((linear_x >> 8) & 0xFF);
    buffer[5] = static_cast<uint8_t>(linear_x & 0xFF);
    buffer[6] = static_cast<uint8_t>((linear_y >> 8) & 0xFF);
    buffer[7] = static_cast<uint8_t>(linear_y & 0xFF);
    buffer[8] = static_cast<uint8_t>((angular_z >> 8) & 0xFF);
    buffer[9] = static_cast<uint8_t>(angular_z & 0xFF);
    buffer[10] = checksum(buffer.data(), buffer.size() - 1U);
    write_bytes(buffer.data(), buffer.size());
  }

  void write_bytes(const uint8_t * data, std::size_t size)
  {
    if (!serial_port_.is_open()) {
      return;
    }
    if (dry_run_) {
      // DRY-RUN: log the first two data bytes (type + length) as a command summary.
      if (size >= 4U) {
        RCLCPP_DEBUG(get_logger(),
          "[DRY-RUN] TX type=0x%02X len=%u", data[3], static_cast<unsigned>(size));
      }
      return;
    }
    std::scoped_lock lock(serial_mutex_);
    try {
      boost::asio::write(serial_port_, boost::asio::buffer(data, size));
    } catch (const std::exception & e) {
      if (!stop_requested_.load()) {
        RCLCPP_ERROR(get_logger(), "Serial write error: %s — port may have disconnected.", e.what());
        needs_reconnect_.store(true);
      }
    }
  }

  void read_exact(uint8_t * destination, std::size_t size)
  {
    // read_exact() is called exclusively from serial_task() (one thread).
    // Locking serial_mutex_ here would deadlock with write_bytes() because
    // boost::asio::read() blocks for the full byte count while the send timer
    // simultaneously tries to acquire the same mutex to write.
    boost::asio::read(serial_port_, boost::asio::buffer(destination, size));
  }

  void serial_task()
  {
    std::array<uint8_t, kMaxFrameSize> frame{};
    uint8_t frame_size = 0;

    enum class State { Head1, Head2, Size, Data, Checksum };
    State state = State::Head1;

    while (!stop_requested_.load() && rclcpp::ok()) {
      // Reconnect logic: triggered by write_bytes() on serial error
      if (needs_reconnect_.load()) {
        needs_reconnect_.store(false);
        RCLCPP_WARN(get_logger(), "Serial port disconnected — attempting reconnect in 1 s...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        try {
          boost::system::error_code ec;
          serial_port_.cancel(ec);
          serial_port_.close(ec);
          open_serial_port();
          send_coefficient();
          send_params();
          state = State::Head1;
          RCLCPP_INFO(get_logger(), "Serial port reconnected successfully.");
        } catch (const std::exception & reconnect_ex) {
          RCLCPP_ERROR(get_logger(), "Reconnect failed: %s", reconnect_ex.what());
          needs_reconnect_.store(true);  // retry next cycle
        }
        continue;
      }

      try {
        switch (state) {
          case State::Head1:
            read_exact(frame.data(), 1);
            state = frame[0] == kHead1 ? State::Head2 : State::Head1;
            break;
          case State::Head2:
            read_exact(frame.data() + 1, 1);
            state = frame[1] == kHead2 ? State::Size : State::Head1;
            break;
          case State::Size:
            read_exact(frame.data() + 2, 1);
            frame_size = frame[2];
            if (frame_size < 5U || frame_size > frame.size()) {
              state = State::Head1;
              break;
            }
            state = State::Data;
            break;
          case State::Data:
            read_exact(frame.data() + 3, frame_size - 4U);
            state = State::Checksum;
            break;
          case State::Checksum:
            read_exact(frame.data() + frame_size - 1U, 1);
            if (frame[frame_size - 1U] == checksum(frame.data(), frame_size - 1U)) {
              handle_frame(frame.data(), frame_size);
            }
            state = State::Head1;
            break;
        }
      } catch (const std::exception & exception) {
        if (!stop_requested_.load()) {
          RCLCPP_ERROR(get_logger(), "Serial receive loop stopped: %s", exception.what());
        }
        break;
      }
    }
  }

  void handle_frame(const uint8_t * data, std::size_t frame_size)
  {
    if (frame_size < 42U) {
      return;
    }

    const auto current_time = now();
    double delta_time = (current_time - last_receive_time_).seconds();
    if (delta_time <= 1e-6) {
      delta_time = 1e-3;
    }
    last_receive_time_ = current_time;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = imu_frame_id_;
    imu_msg.angular_velocity.x = static_cast<double>(decode_int16(data[4], data[5])) / 32768.0 * 2000.0 / 180.0 * M_PI;
    imu_msg.angular_velocity.y = static_cast<double>(decode_int16(data[6], data[7])) / 32768.0 * 2000.0 / 180.0 * M_PI;
    imu_msg.angular_velocity.z = static_cast<double>(decode_int16(data[8], data[9])) / 32768.0 * 2000.0 / 180.0 * M_PI;
    imu_msg.linear_acceleration.x = static_cast<double>(decode_int16(data[10], data[11])) / 32768.0 * 2.0 * 9.8;
    imu_msg.linear_acceleration.y = static_cast<double>(decode_int16(data[12], data[13])) / 32768.0 * 2.0 * 9.8;
    imu_msg.linear_acceleration.z = static_cast<double>(decode_int16(data[14], data[15])) / 32768.0 * 2.0 * 9.8;
    const auto yaw_degrees = static_cast<double>(decode_int16(data[20], data[21])) / 10.0;
    tf2::Quaternion imu_quaternion;
    imu_quaternion.setRPY(0.0, 0.0, yaw_degrees / 180.0 * M_PI);
    imu_msg.orientation = tf2::toMsg(imu_quaternion);
    imu_msg.orientation_covariance = {1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 0.02};
    imu_msg.angular_velocity_covariance = {1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e-4};
    imu_msg.linear_acceleration_covariance = {1e-2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_pub_->publish(imu_msg);

    const double odom_x = static_cast<double>(decode_int16(data[22], data[23])) / 1000.0;
    const double odom_y = static_cast<double>(decode_int16(data[24], data[25])) / 1000.0;
    const double odom_yaw = static_cast<double>(decode_int16(data[26], data[27])) / 1000.0;
    const double delta_x = static_cast<double>(decode_int16(data[28], data[29])) / 1000.0;
    const double delta_y = static_cast<double>(decode_int16(data[30], data[31])) / 1000.0;
    const double delta_yaw = static_cast<double>(decode_int16(data[32], data[33])) / 1000.0;

    tf2::Quaternion odom_quaternion;
    odom_quaternion.setRPY(0.0, 0.0, odom_yaw);
    geometry_msgs::msg::Quaternion odom_quaternion_msg = tf2::toMsg(odom_quaternion);

    if (publish_odom_transform_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = current_time;
      transform.header.frame_id = odom_frame_id_;
      transform.child_frame_id = base_frame_id_;
      transform.transform.translation.x = odom_x;
      transform.transform.translation.y = odom_y;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = odom_quaternion_msg;
      tf_broadcaster_->sendTransform(transform);
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quaternion_msg;
    odom_msg.twist.twist.linear.x = delta_x / delta_time;
    odom_msg.twist.twist.linear.y = 0.0; // Ackermann kinematic constraint: no lateral velocity
    odom_msg.twist.twist.angular.z = delta_yaw / delta_time;
    odom_msg.twist.covariance = {1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
                                 0.0,  1e6,  1e6,  0.0,  0.0,  0.0,
                                 0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
                                 0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
                                 0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
                                 0.0,  0.0,  0.0,  0.0,  0.0,  1e-3};
    odom_msg.pose.covariance = {1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1e3};
    odom_pub_->publish(odom_msg);

    publish_motor_value(left_velocity_pub_, decode_int16(data[34], data[35]));
    publish_motor_value(right_velocity_pub_, decode_int16(data[36], data[37]));
    publish_motor_value(left_setpoint_pub_, decode_int16(data[38], data[39]));
    publish_motor_value(right_setpoint_pub_, decode_int16(data[40], data[41]));

    const double voltage = static_cast<double>(decode_int16(data[16], data[17])) / 1000.0;
    const double current = static_cast<double>(decode_int16(data[18], data[19])) / 1000.0;
    publish_battery_state(voltage, current);

    const double rear_left_linear_speed =
      static_cast<double>(decode_int16(data[34], data[35])) / 1000.0;
    const double rear_right_linear_speed =
      static_cast<double>(decode_int16(data[36], data[37])) / 1000.0;
    publish_joint_state(current_time, rear_left_linear_speed, rear_right_linear_speed, delta_time);
  }

  void publish_battery_state(double voltage, double current)
  {
    // Cache current measurement for diagnostics and brownout prediction
    last_battery_current_ = current;

    // Battery model: OCV = V_measured + (I_load × R_internal)
    // This compensates for voltage sag under current draw
    double ocv_estimated = voltage + (current * battery_internal_resistance_ / 1000.0);

    // Percentage based on OCV model (12.6V full, 9.0V empty for 3S 18650)
    double percentage_ocv = (ocv_estimated - 9.0) / (12.6 - 9.0);
    double percentage_clamped = std::max(0.0, std::min(1.0, percentage_ocv));

    // Brownout prediction: What is minimum voltage if we draw peak current (3A)?
    double peak_predicted_current = 3000.0;  // mA: typical Nano peak during vision inference
    double min_voltage_at_peak = ocv_estimated - (peak_predicted_current * battery_internal_resistance_ / 1000.0);

    // Health determination
    uint8_t health_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    if (min_voltage_at_peak < battery_abort_threshold_) {
      health_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
      if (brownout_warning_count_ < 100) {
        RCLCPP_ERROR(
          get_logger(),
          "CRITICAL: Battery will brownout at peak load. OCV=%.2fV, min_peak=%.2fV < abort_threshold=%.2fV",
          ocv_estimated, min_voltage_at_peak, battery_abort_threshold_);
        brownout_warning_count_++;
      }
    } else if (min_voltage_at_peak < battery_derating_threshold_) {
      health_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WARM;
      if (brownout_warning_count_ < 50) {
        RCLCPP_WARN(
          get_logger(),
          "WARNING: Battery degraded. OCV=%.2fV, min_peak=%.2fV. Recommend speed derating.",
          ocv_estimated, min_voltage_at_peak);
        brownout_warning_count_++;
      }
    } else if (voltage < battery_brownout_threshold_) {
      health_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WARM;
    }

    // Construct and publish BatteryState message
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.voltage = static_cast<float>(voltage);
    msg.current = static_cast<float>(current);
    msg.design_capacity = 2.6; // 2600mAh typical 18650
    msg.percentage = static_cast<float>(percentage_clamped);
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = health_status;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    battery_pub_->publish(msg);
    last_battery_voltage_ = voltage;
  }

  void publish_joint_state(
    const rclcpp::Time & stamp,
    double rear_left_linear_speed,
    double rear_right_linear_speed,
    double delta_time)
  {
    if (wheel_radius_ <= 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "wheel_radius must be positive to publish joint_states.");
      return;
    }

    const double rear_left_angular_speed = rear_left_linear_speed / wheel_radius_;
    const double rear_right_angular_speed = rear_right_linear_speed / wheel_radius_;

    rear_left_wheel_position_ += rear_left_angular_speed * delta_time;
    rear_right_wheel_position_ += rear_right_angular_speed * delta_time;

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    // Only publish joints backed by measured wheel telemetry from the RP2040.
    msg.name = {"rear_left_wheel_joint", "rear_right_wheel_joint"};
    msg.position = {rear_left_wheel_position_, rear_right_wheel_position_};
    msg.velocity = {rear_left_angular_speed, rear_right_angular_speed};
    joint_pub_->publish(msg);
  }

  void publish_motor_value(
    const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr & publisher,
    int16_t value)
  {
    std_msgs::msg::Int32 msg;
    msg.data = value;
    publisher->publish(msg);
  }

  void diagnostic_battery(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    // Estimate OCV from loaded measurement.
    double ocv_estimated = last_battery_voltage_ + (last_battery_current_ * battery_internal_resistance_ / 1000.0);
    double min_voltage_at_peak = ocv_estimated - (3000.0 * battery_internal_resistance_ / 1000.0);

    std::string status_msg;
    if (min_voltage_at_peak < battery_abort_threshold_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "BROWNOUT CRITICAL");
      status_msg = "Will dropout at peak load";
    } else if (min_voltage_at_peak < battery_derating_threshold_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low Battery - Derate Speed");
      status_msg = "Degraded, recommend 50% speed limit";
    } else if (last_battery_voltage_ < battery_brownout_threshold_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low Battery");
      status_msg = "Voltage low, current OK";
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery Healthy");
      status_msg = "Normal";
    }

    stat.add("Voltage (V)", last_battery_voltage_);
    stat.add("Current (mA)", last_battery_current_);
    stat.add("OCV Estimated (V)", ocv_estimated);
    stat.add("Min at 3A Peak (V)", min_voltage_at_peak);
    stat.add("Status", status_msg);
    stat.add("Brownout Warnings", static_cast<int>(brownout_warning_count_));
  }

  void diagnostic_rp2040(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    // Monitor RP2040 firmware health
    uint8_t health = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string health_msg = "Nominal";

    if (rp2040_telemetry_.watchdog_resets_count > rp2040_watchdog_reset_threshold_) {
      health = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      health_msg = "Excessive watchdog resets";
    }

    if (rp2040_telemetry_.command_ack_rate < 95) {
      health = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      health_msg = "Low command acknowledgement rate";
    }

    stat.summary(health, health_msg);

    stat.add("Firmware Version", static_cast<int>(rp2040_telemetry_.firmware_version));
    stat.add("Watchdog Resets", static_cast<int>(rp2040_telemetry_.watchdog_resets_count));
    stat.add("E-Stop Reason", static_cast<int>(rp2040_telemetry_.estop_reason));
    stat.add("Command ACK Rate (%)", static_cast<int>(rp2040_telemetry_.command_ack_rate));
    stat.add("Steering Angle (rad)", static_cast<double>(rp2040_telemetry_.steering_angle_measured) / 1000.0);
    stat.add("3.3V Rail (mV)", static_cast<int>(rp2040_telemetry_.voltage_rail_3v3));
    stat.add("5.0V Rail (mV)", static_cast<int>(rp2040_telemetry_.voltage_rail_5v0));
    stat.add("Total Commands", static_cast<int>(rp2040_command_count_));
    stat.add("Total Acks", static_cast<int>(rp2040_ack_count_));
  }

  void diagnostic_serial(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (serial_port_.is_open()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Port Open");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Port Closed");
    }
  }

  void diagnostic_heartbeat(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const double age = (now() - last_command_time_).seconds();
    if (age < command_timeout_sec_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Active");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Timed Out (Braking)");
    }
    stat.add("Last Command Age (s)", age);
    stat.add("Timeout Threshold (s)", command_timeout_sec_);
  }

  void diagnostic_sensors(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const double age = (now() - last_receive_time_).seconds();
    if (age < 0.2) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Healthy");
    } else if (age < 1.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Stale Data");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No Data (Hardware Link Failed)");
    }
    stat.add("Stream Latency (s)", age);
  }

  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  std::mutex serial_mutex_;
  std::mutex command_mutex_;
  std::thread serial_thread_;
  std::atomic<bool> stop_requested_{false};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_setpoint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  std::string port_name_;
  int baud_rate_{};
  bool publish_odom_transform_{};
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string imu_frame_id_;
  double linear_correction_{};
  double coefficient_a_{};
  double coefficient_b_{};
  double coefficient_c_{};
  double coefficient_d_{};
  int kp_{};
  int ki_{};
  int kd_{};
  int servo_bias_{};
  double command_timeout_sec_{};
  double send_period_sec_{};
  double wheel_radius_{};
  double max_linear_velocity_{};
  double max_lateral_velocity_{};
  double max_steering_angle_{};

  double commanded_x_{0.0};
  double commanded_y_{0.0};
  double commanded_yaw_{0.0};
  double prev_command_x_{0.0};  // For acceleration ramp
  rclcpp::Time last_command_time_;
  rclcpp::Time last_receive_time_;
  rclcpp::Time node_start_time_;

  // Safety additions
  bool dry_run_{false};
  double max_accel_mps2_{0.5};
  double startup_speed_limit_ms_{0.20};
  double startup_duration_sec_{30.0};
  std::atomic<bool> needs_reconnect_{false};
  double last_battery_voltage_{12.6};
  double last_battery_current_{0.0};
  double battery_internal_resistance_{0.5};  // Ohms: typical 18650 3S
  double battery_brownout_threshold_{9.6};   // Minimum safe voltage (Jetson spec)
  double battery_derating_threshold_{10.0};  // Below this: derate to 50% speed
  double battery_abort_threshold_{9.3};      // Below this: abort mission immediately
  uint32_t brownout_warning_count_{0};       // Track warnings for diagnostics
  
  // RP2040 telemetry tracking
  RP2040Telemetry rp2040_telemetry_{};
  uint32_t rp2040_command_count_{0};
  uint32_t rp2040_ack_count_{0};
  uint32_t rp2040_watchdog_reset_threshold_{5};  // Warn if > 5 resets/hour
  
  double rear_left_wheel_position_{0.0};
  double rear_right_wheel_position_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<JetRacerSerialNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    fprintf(stderr, "jetracer_hardware failed: %s\n", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
