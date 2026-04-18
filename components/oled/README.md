# 0.91" I2C OLED Telemetry Module

This directory contains standalone diagnostics for the JetRacer's onboard OLED screen.

## Architecture
The OLED is an SSD1306-driven 128x32 pixel display wired into the JetRacer's shared `I2C` communication bus. It serves as a rapid, headless debug module. When the system boots, it automatically queries the underlying Ubuntu layer and eth0/wlan0 pipelines to broadcast the live IP Address, Memory consumption, and Disk utilization.

This ensures you can always SSH into your robot without needing an external monitor to determine its dynamically assigned IP address!

## Hardware Diagnostic
You can validate the I2C pixel-painting functionality independently of the global OS boot scripts by running the raw driver test:

```bash
python3 jetson_oled_test.py
```

### Expected Output
The screen will clear, initializing the `adafruit_ssd1306` library, and immediately render diagnostic metrics on the crystal display. 

> [!TIP]
> If you encounter `I2C Bus Error: Timeout` locks, ensure your `docker-compose.yaml` and local Host OS are not trying to paint the screen simultaneously. I2C is not thread-safe over multiple concurrent masters.
