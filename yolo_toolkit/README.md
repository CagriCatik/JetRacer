# 🚀 JetRacer YOLO Training Toolkit

This is a specialized training suite designed for high-precision traffic sign detection on the JetRacer platform.

## 🧠 Expert Features
- **Orientation-Preserving Augmentation**: Our configurations explicitly disable horizontal flips. This ensures your model can distinguish between "Left Turn" and "Right Turn" signs, which is a common failure point in default YOLO setups.
- **Decoupled Configuration**: All training parameters (learning rate, mosaic, batch size) are controlled via `configs/hyperparameters.yaml`—**never touch the code**.
- **Quantization Ready**: The pipeline is pre-configured to export models optimized for the Jetson Nano's TensorRT engines.

## 🛠️ Setup & Installation

Follow these steps to set up an isolated training environment on your PC (Windows or Linux).

### 1. Create Virtual Environment
```bash
# Navigate to the toolkit directory
cd yolo_toolkit

# Create the venv
python -m venv venv
```

### 2. Activate Environment
**Windows:**
```bash
.\venv\Scripts\activate
```
**Linux / macOS:**
```bash
source venv/bin/activate
```

### 3. Install Expert Dependencies
```bash
# Upgrade pip first
python -m pip install --upgrade pip

# Install requirements
pip install -r requirements.txt
```

---

## 🚀 Execution Guide
Adjust your settings in `configs/hyperparameters.yaml` and then run:

### 1. Training
```bash
python main.py train
```
Output models will appear in `outputs/jetracer_hardened/`.

### 2. Export for Jetson
Convert your best model to a high-speed FP16 `.engine` file:
```bash
python main.py export --weights outputs/jetracer_hardened/weights/best.pt
```

### 3. Validation
Verify the accuracy (mAP) and speed of your model (works for both `.pt` and `.engine`):
```bash
# Validate on validation set
python main.py validate --weights outputs/jetracer_hardened/weights/best.pt

# Validate on final test set
python main.py validate --weights outputs/jetracer_hardened/weights/best.pt --split test
```

---
> [!IMPORTANT]
> **Why disable `fliplr`?**
> Standard YOLO training "flips" images to create variations. If the model flips a "Turn Right" sign, it becomes a "Turn Left" sign but retains the "Turn Right" label. This confuses the AI. Our expert config prevents this!
