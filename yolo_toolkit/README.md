# 🚀 JetRacer YOLO Training Toolkit

This is a specialized training suite designed for high-precision traffic sign detection on the JetRacer platform.

## 🧠 Expert Features
- **Orientation-Preserving Augmentation**: Our configurations explicitly disable horizontal flips. This ensures your model can distinguish between "Left Turn" and "Right Turn" signs, which is a common failure point in default YOLO setups.
- **Decoupled Configuration**: All training parameters (learning rate, mosaic, batch size) are controlled via `configs/hyperparameters.yaml`—**never touch the code**.
- **Quantization Ready**: The pipeline is pre-configured to export models optimized for the Jetson Nano's TensorRT engines.

## 🛠️ Usage

### 1. Requirements
Ensure you are in a Python 3.10+ environment with CUDA available.
```bash
pip install -r yolo_toolkit/requirements.txt
```

### 2. Training
Adjust your settings in `yolo_toolkit/configs/hyperparameters.yaml` and then run:
```bash
python yolo_toolkit/core/train_yolo.py --data dataset/data.yaml
```
Output models will appear in `yolo_toolkit/outputs/jetracer_hardened/`.

### 3. Export for Jetson
Convert your best model to a high-speed FP16 `.engine` file:
```bash
python yolo_toolkit/core/export_yolo.py --weights yolo_toolkit/outputs/jetracer_hardened/weights/best.pt --half
```

---
> [!IMPORTANT]
> **Why disable `fliplr`?**
> Standard YOLO training "flips" images to create variations. If the model flips a "Turn Right" sign, it becomes a "Turn Left" sign but retains the "Turn Right" label. This confuses the AI. Our expert config prevents this!
