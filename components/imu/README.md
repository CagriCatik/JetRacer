# BNO085 AHRS Internal Measurement Unit (IMU)

This directory contains standalone diagnostics for the JetRacer's hardware IMU, isolated entirely from the ROS 2 autonomous stack.

## Architecture & Integration
The IMU sits across the native `I2C` bus. Unlike cheap, raw accelerometers, the BNO085 acts as an AHRS (Attitude and Heading Reference System). This means it physically contains a burned-in embedded processor that fuses the accelerometer, gyroscope, and magnetometer locally using an internal Kalman filter. 

It passes pre-fused, highly stable **Euler Angles** directly to the Jetson Nano, preventing computationally expensive drift-correction on the host CPU.

## Hardware Diagnostic
If you suspect the EKF orientation in `robot_localization` is dropping, you can test the bus integrity at the silicon level.

```bash
# Execute within an isolated Python environment
python3 jetson_imu_test.py
```

### Expected Output
The console will flood with real-time Roll, Pitch, and Yaw matrices. When the vehicle is perfectly flat, Roll and Pitch should stabilize near absolute `0.00`. Rotation of the vehicle should actively swing the Yaw axis cleanly from `-180` to `180` degrees.

> [!WARNING]
> Do **NOT** run this script while the Docker `twist_mux` autonomous stack is active! `smbus2` will crash violently if two different processes attempt to lock the `/dev/i2c-1` register at the exact same microsecond!
