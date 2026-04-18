# Sony IMX219 (8MP) CSI Camera

This directory validates the raw ribbon-cable connectivity of the IMX219 optical sensor.

## Architecture
Unlike standard USB webcams that process image encoding internally and suffer from massive bandwidth limits on the USB bridge, the IMX219 feeds directly into the Jetson Nano's **CSI (Camera Serial Interface)**.

This pipeline leverages the physical NVIDIA Image Signal Processor (ISP) internally built onto the Maxwell GPU array, capturing incredibly fast $640 \times 480$ matrices without dropping computational cycles on the generalized CPU.

## Hardware Diagnostic
This script utilizes raw GStreamer pipelines to test the sensor completely independently of the `v4l2_camera` ROS 2 nodes.

```bash
python3 csi_cam_test.py
```

### Expected Output
It will initialize a GStreamer pipeline and render a standard graphical desktop window plotting 30 FPS RGB buffers.

> [!WARNING]
> This diagnostic requires an active X11 graphical interface (a monitor plugged into the Jetson Nano, or an X-Forwarded SSH connection). You cannot spawn the `cv2.imshow()` renderer natively in a pure headless SSH shell without throwing a `Gtk-WARNING`.
