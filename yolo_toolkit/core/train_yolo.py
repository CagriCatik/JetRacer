#!/usr/bin/env python3
"""
JetRacer Expert YOLO Training Engine
Decoupled architecture: Logic in Core, Tuning in Configs.
"""

import os
import yaml
import argparse
from ultralytics import YOLO
from pathlib import Path

def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def main():
    parser = argparse.ArgumentParser(description="JetRacer Expert Training Wrapper")
    parser.add_argument("--config", type=str, default="yolo_toolkit/configs/hyperparameters.yaml", help="Path to hyperparams YAML")
    parser.add_argument("--data", type=str, default="dataset/data.yaml", help="Path to dataset YAML")
    parser.add_argument("--output", type=str, default="yolo_toolkit/outputs", help="Output root")
    
    args = parser.parse_args()
    
    # 1. Load Expert Config
    if not os.path.exists(args.config):
        print(f"Error: Config file {args.config} not found.")
        return
        
    cfg = load_config(args.config)
    print(f"\n[EXPERT INFO] Initializing training with {cfg['model']} for {cfg['epochs']} epochs.")
    if cfg.get('fliplr', 1.0) == 0.0:
        print("[SAFETY NOTICE] Horizontal Flip is DISABLED to protect directional sign labels.")

    # 2. Setup Directories
    Path(args.output).mkdir(parents=True, exist_ok=True)

    # 3. Initialize Model
    model = YOLO(cfg['model'])

    # 4. Run Training using the full config dictionary
    # We unpack the dictionary into the train function
    results = model.train(
        data=args.data,
        project=args.output,
        name="jetracer_hardened",
        **cfg # Expert injection of all YAML params
    )

    print(f"\n--- SUCCESS ---")
    print(f"Best Weights: {args.output}/jetracer_hardened/weights/best.pt")
    print(f"To export for Jetson, use: python yolo_toolkit/core/export_yolo.py --weights {args.output}/jetracer_hardened/weights/best.pt --half")

if __name__ == "__main__":
    main()
