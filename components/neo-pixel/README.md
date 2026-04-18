# WS2812 RGB Expansion Ring (NeoPixels)

This directory contains standalone diagnostics for the LED Neopixel ring often mounted to the front bumper of the JetRacer. 

## Architecture
These RGB LEDs operate via Pulse Width Modulation (PWM). In many consumer DIY robots, the LEDs are tied to the primary OS (blinking to indicate CPU load). However, the precision requirements of WS2812 timing means that if arbitrary Linux background processes interrupt the driver loop, the LED array will crash or color-shift violently.

Our Jetson Nano delegates PWM timing seamlessly through its internal GPIO matrix.

## Hardware Diagnostic
You can validate the RGB matrix dynamically using the underlying Python peripheral library.

```bash
python3 neopixel-test.py
```

### Expected Output
The script injects specific hex values into the controller. You will observe the ring sweep flawlessly across pure Red, Green, and Blue spectrums before engaging a mathematical rainbow animation sequence. 

> [!WARNING]
> Because driving NeoPixels directly from the Jetson GPIO demands strict root-level hardware DMA (Direct Memory Access) permissions, you must normally execute NeoPixel scripts with `sudo` natively, which necessitates aggressive `--privileged` modes if trying to run them from inside a localized Docker container!
