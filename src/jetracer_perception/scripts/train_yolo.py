#!/usr/bin/env python3
import os
import argparse
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(description="JetRacer YOLO Training Script")
    parser.add_argument("--data", type=str, default="dataset/data.yaml", help="Path to data.yaml")
    parser.add_argument("--model", type=str, default="yolo11n.pt", help="Base model to train (yolo11n.pt, yolo8n.pt)")
    parser.add_argument("--epochs", type=int, default=50, help="Number of training epochs")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--project", type=str, default="models/training", help="Project output directory")
    parser.add_argument("--name", type=str, default="jetracer_signs", help="Experiment name")
    
    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.project, exist_ok=True)

    # Initialize model
    print(f"--- Initializing YOLO training with base: {args.model} ---")
    model = YOLO(args.model)

    # Start training
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name=args.name,
        device=0, # Use first GPU
        exist_ok=True,
        cache=True # Faster training if RAM allows
    )

    print(f"--- Training Complete! Best model saved to: {args.project}/{args.name}/weights/best.pt ---")

if __name__ == "__main__":
    main()
