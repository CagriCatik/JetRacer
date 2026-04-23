# JetRacer YOLO Training Toolkit

A professional, hardware-optimized training and deployment suite for the JetRacer autonomous platform. This toolkit is engineered to bridge the gap between model training and real-world robotics execution.

---

## Industrial-Grade Features

*   **Orientation-Aware Augmentation**: Our configurations explicitly disable horizontal flips (`fliplr: 0.0`). This is critical for traffic signs (e.g., distinguishing Left vs. Right turns), preventing the label poisoning common in default YOLO pipelines.
*   **Dataset Intelligence**: Built-in auditing tools to identify class imbalances and label density issues before expensive training runs.
*   **TensorRT Hardware Optimization**: Integrated support for high-performance `.engine` export, enabling real-time inference on the Jetson Nano and RTX 3080.
*   **Unified CLI**: A single-entry command-line interface for the entire lifecycle: Analysis → Training → Validation → Benchmarking → Deployment.
*   **Quantization Ready**: Pre-configured for FP16 quantization to maximize throughput with minimal precision loss.

---

## Dataset Intelligence

The toolkit is optimized for the following traffic intelligence classes:

| ID | Class Name | Description |
| :--- | :--- | :--- |
| 0-1 | `30_ZONE_BEGIN/END` | Speed zone boundaries |
| 2 | `ATTENTION` | General hazard warning |
| 3 | `CHILD` | Pedestrian/School zone safety |
| 4 | `GIVE_WAY` | Yield priority control |
| 5 | `PERSON` | Human detection |
| 6 | `PRIORITY_ROAD` | ROW assertion |
| 7 | `RIGHT_TURN` | Mandatory direction |
| 8 | `STOP` | Full halt trigger (Highest Priority) |

**Source:** [Roboflow Project: final-jetracer-traffic-signs](https://universe.roboflow.com/jetson-z26yf/final-jetracer-traffic-signs/dataset/9)

---

## Setup & Environmental Hardening

Follow these steps to establish a high-performance training environment. **Python 3.11** is the recommended runtime.

### 1. Initialize Environment
```powershell
# Navigate and create virtual environment
cd yolo_toolkit
python -m venv venv
.\venv\Scripts\activate
```

### 2. High-Performance Dependencies
```powershell
# Install GPU-Accelerated PyTorch (RTX 30 series optimized)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install Expert Suite & TensorRT Optimization
pip install -r requirements.txt
```

---

## Unified Execution Suite

### 1. Pre-Flight Analysis
Scan your dataset for health metrics and class distribution:
```powershell
python main.py analyze --data ../dataset/data.yaml
```

### 2. Expert Training
Launch the hardware-optimized training engine:
```powershell
python main.py train --config configs/hyperparameters.yaml
```
> [!NOTE]
> All results are consolidated into the established `outputs/` directory for clean management.

### 3. Hardware Benchmarking
Verify real-world FPS, latency, and jitter directly on your target hardware:
```powershell
python main.py benchmark --weights outputs/jetracer_hardened/weights/best.pt
```

### 4. TensorRT Deployment (Engine Export)
Convert your model to a high-speed hardware engine for the Jetson Nano:
```powershell
python main.py export --weights outputs/jetracer_hardened/weights/best.pt --half
```

---

> [!IMPORTANT]
> **Expert Tip: The "Orientation" Rule**
> Standard YOLO pipelines use horizontal flips as a "general" augmentation. In robotics, this is dangerous: a "Turn Right" sign becomes a "Turn Left" sign but retains the "Turn Right" label. Our `configs/hyperparameters.yaml` explicitly enforces directional integrity.
