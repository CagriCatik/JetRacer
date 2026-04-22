#!/usr/bin/env python3
"""
JetRacer Expert YOLO Validation Engine
Supports .pt and .engine (TensorRT) models.
"""

import argparse
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(description="JetRacer YOLO Validation Script")
    parser.add_argument("--weights", type=str, required=True, help="Path to best.pt or best.engine")
    parser.add_argument("--data", type=str, default="dataset/data.yaml", help="Path to data.yaml")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--split", type=str, default="val", choices=["val", "test"], help="Dataset split to validate on")
    
    args = parser.parse_args()

    print(f"--- Firing up Validation for: {args.weights} ---")
    
    # Load model
    model = YOLO(args.weights)

    # Run validation
    metrics = model.val(
        data=args.data,
        imgsz=args.imgsz,
        batch=args.batch,
        split=args.split,
        device=0 # Use GPU
    )

    print("\n" + "="*30)
    print("      VALIDATION RESULTS      ")
    print("="*30)
    print(f"mAP50:    {metrics.box.map50:.4f}")
    print(f"mAP50-95: {metrics.box.map:.4f}")
    print(f"Inference: {metrics.speed['inference']:.2f}ms")
    print(f"Preprocess: {metrics.speed['preprocess']:.2f}ms")
    print("="*30)

if __name__ == "__main__":
    main()
