#!/usr/bin/env python3
import argparse
import sys
import os

# Add core to path so we can import modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'core'))

from train_yolo import main as train_main
from export_yolo import main as export_main

def main():
    parser = argparse.ArgumentParser(description="JetRacer YOLO Toolkit - Unified Interface")
    subparsers = parser.add_subparsers(dest="mode", help="Execution mode")

    # Train Subcommand
    train_parser = subparsers.add_parser("train", help="Start training on the dataset")
    train_parser.add_argument("--config", type=str, default="configs/hyperparameters.yaml", help="Hyperparams config")
    train_parser.add_argument("--data", type=str, default="../dataset/data.yaml", help="Dataset config")

    # Export Subcommand
    export_parser = subparsers.add_parser("export", help="Export weights to Engine/ONNX")
    export_parser.add_argument("--weights", type=str, required=True, help="Path to .pt weights")
    export_parser.add_argument("--half", action="store_true", default=True, help="Use FP16 (Recommended)")

    args = parser.parse_args()

    if args.mode == "train":
        # Pass synthetic arguments to the core train script
        sys.argv = [sys.argv[0], "--config", args.config, "--data", args.data]
        train_main()
    
    elif args.mode == "export":
        # Pass synthetic arguments to the core export script
        sys.argv = [sys.argv[0], "--weights", args.weights]
        if args.half:
            sys.argv.append("--half")
        export_main()
    
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
