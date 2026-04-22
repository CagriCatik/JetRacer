#!/usr/bin/env python3
import argparse
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(description="JetRacer YOLO Export Script")
    parser.add_argument("--weights", type=str, required=True, help="Path to best.pt weights")
    parser.add_argument("--format", type=str, default="engine", help="Format to export (engine, onnx, tflite)")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--half", action="store_true", help="Use FP16 quantization (Recommended for Jetson)")
    parser.add_argument("--int8", action="store_true", help="Use INT8 quantization (Requires calibration data)")
    
    args = parser.parse_args()

    print(f"--- Exporting {args.weights} to {args.format} ---")
    model = YOLO(args.weights)

    # Export
    path = model.export(
        format=args.format,
        imgsz=args.imgsz,
        half=args.half,
        int8=args.int8,
        simplify=True # Optimize ONNX graph
    )

    print(f"--- Export Complete! Model saved to: {path} ---")

if __name__ == "__main__":
    main()
