# JetRacer Traffic Signs Dataset

## Overview

This dataset is designed for training and evaluating object detection models on traffic signs for JetRacer-based autonomous driving projects. It contains labeled images of common road signs relevant to small-scale robotics and embedded AI applications.

## Dataset Structure

The dataset is organized into three standard splits:

```shell
dataset/
├── train/
│   └── images/
├── valid/
│   └── images/
├── test/
│   └── images/
```

### Dataset Path

For stability (especially on Windows), an absolute path can be used:

```shell
<DATASET_PATH>
```

## Classes

The dataset includes **9 traffic sign classes**:

* 30_ZONE_BEGIN
* 30_ZONE_END
* ATTENTION
* CHILD
* GIVE_WAY
* PERSON
* PRIORITY_ROAD
* RIGHT_TURN
* STOP

## Configuration (YOLO Format)

```yaml
path: <DATASET_PATH>
train: train/images
val: valid/images
test: test/images

nc: 9
names: ['30_ZONE_BEGIN', '30_ZONE_END', 'ATTENTION', 'CHILD', 'GIVE_WAY', 'PERSON', 'PRIORITY_ROAD', 'RIGHT_TURN', 'STOP']
```

## Source

This dataset was created and exported using Roboflow:

* **Workspace:** jetson-z26yf
* **Project:** final-jetracer-traffic-signs
* **Version:** 9
* **License:** CC BY 4.0

Access it online:
[https://universe.roboflow.com/jetson-z26yf/final-jetracer-traffic-signs/dataset/9](https://universe.roboflow.com/jetson-z26yf/final-jetracer-traffic-signs/dataset/9)

## License

This dataset is licensed under the **Creative Commons Attribution 4.0 (CC BY 4.0)** license.
You are free to use, modify, and distribute it, provided proper attribution is given.

## Usage

This dataset is suitable for:

* Training YOLO-based object detection models
* JetRacer autonomous driving experiments
* Embedded AI (Jetson devices)
* Real-time traffic sign recognition

## Notes

* Replace `<DATASET_PATH>` with your actual dataset location.
* For Linux environments, use a Unix-style path (e.g., `/home/user/dataset`).
* Labels follow the standard YOLO annotation format.
