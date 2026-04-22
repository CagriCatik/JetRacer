# 🚀 JetRacer YOLO Training Toolkit

This is a professional, industrial-grade training suite designed for high-precision traffic sign detection on the JetRacer platform.

## 🧠 Expert Features
- **Orientation-Preserving Augmentation**: Our configurations explicitly disable horizontal flips. This ensures your model can distinguish between "Left Turn" and "Right Turn" signs—a common failure point in default YOLO setups.
- **Dataset Intelligence**: Built-in auditing to identify class imbalances (e.g., too many "Stop" signs vs not enough "Person" labels) before you waste time training.
- **Hardware Benchmarking**: Quantitatively verify your real-world FPS and inference jitter on the 3080 or the Jetson Nano.
- **Quantization Ready**: Pre-configured for high-speed FP16 TensorRT deployment.

## 🛠️ Setup & Installation (RTX 3080 Optimized)

Follow these steps to set up your high-performance training environment. **Python 3.11 is required.**

### 1. Create Virtual Environment
```bash
# Navigate to the toolkit directory
cd yolo_toolkit

# Create the venv (Use your Python 3.11 path)
& "C:\Users\mccat\AppData\Local\Programs\Python\Python311\python.exe" -m venv venv
```

### 2. Activate & Install
```bash
# Activate
.\venv\Scripts\activate

# 1. Install GPU-Accelerated PyTorch (For RTX 3080)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 2. Install Expert Toolkit
pip install -r requirements.txt
```

---

## 🚀 Execution Guide (The Unified CLI)

### 1. Analyze Dataset Health
Always run this first to check for class imbalance:
```bash
python main.py analyze
```

### 2. Start Training
Adjust your settings in `configs/hyperparameters.yaml` and then run:
```bash
python main.py train
```
*Results save to `outputs/jetracer_hardened/`.*

### 3. Validate & Benchmark
Verify accuracy and real-world speed:
```bash
# Check accuracy (mAP)
python main.py validate --weights outputs/jetracer_hardened/weights/best.pt

# Check hardware FPS/Latency
python main.py benchmark --weights outputs/jetracer_hardened/weights/best.pt
```

### 4. Export for Jetson
```bash
python main.py export --weights outputs/jetracer_hardened/weights/best.pt
```

---
> [!IMPORTANT]
> **Expert Tip: The "Orientation" Rule**
> Standard YOLO training "flips" images. If the model flips a "Turn Right" sign, it becomes a "Turn Left" sign but retains the "Turn Right" label. Our expert config in `configs/hyperparameters.yaml` sets `fliplr: 0.0` to prevent this data poisoning.
