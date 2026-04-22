#!/usr/bin/env python3
import argparse
import sys
import os
from rich.console import Console
from rich.panel import Panel
from rich.text import Text

# Aesthetic Console
console = Console()

# Add core to path so we can import modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'core'))

from train_yolo import main as train_main
from export_yolo import main as export_main
from validate_yolo import main as validate_main

def show_banner():
    banner = Text("🚀 JETRACER YOLO TOOLKIT", style="bold magenta", justify="center")
    console.print(Panel(banner, border_style="cyan"))

def main():
    show_banner()
    
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

    # Validate Subcommand
    validate_parser = subparsers.add_parser("validate", help="Validate model on val/test split")
    validate_parser.add_argument("--weights", type=str, required=True, help="Path to best.pt or best.engine")
    validate_parser.add_argument("--split", type=str, default="val", choices=["val", "test"], help="Dataset split")

    args = parser.parse_args()

    if args.mode == "train":
        console.print("[bold green]Starting Training Mission...[/bold green]")
        sys.argv = [sys.argv[0], "--config", args.config, "--data", args.data]
        train_main()
    
    elif args.mode == "export":
        console.print(f"[bold yellow]Initiating Export for {args.weights}...[/bold yellow]")
        sys.argv = [sys.argv[0], "--weights", args.weights]
        if args.half:
            sys.argv.append("--half")
        export_main()

    elif args.mode == "validate":
        console.print(f"[bold blue]Launching Validation on '{args.split}' split...[/bold blue]")
        sys.argv = [sys.argv[0], "--weights", args.weights, "--split", args.split]
        validate_main()
    
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
