#!/usr/bin/env python3
"""
JetRacer Expert YOLO Validation Engine (Rich-Enabled)
"""

import argparse
from ultralytics import YOLO
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

console = Console()

def main():
    parser = argparse.ArgumentParser(description="JetRacer YOLO Validation Script")
    parser.add_argument("--weights", type=str, required=True, help="Path to best.pt or best.engine")
    parser.add_argument("--data", type=str, default="../dataset/data.yaml", help="Path to data.yaml")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--split", type=str, default="val", choices=["val", "test"], help="Dataset split to validate on")
    
    args = parser.parse_args()

    console.print(f"[bold blue]Processing weights:[/bold blue] [cyan]{args.weights}[/cyan]")
    
    # Load model
    model = YOLO(args.weights)

    # Run validation
    metrics = model.val(
        data=args.data,
        imgsz=args.imgsz,
        batch=args.batch,
        split=args.split,
        device=0 
    )

    # Result Table
    table = Table(title=f"Validation Metrics ({args.split})", title_style="bold magenta")
    table.add_column("Metric", style="cyan")
    table.add_column("Score / Time", style="green", justify="right")

    table.add_row("mAP@50", f"{metrics.box.map50:.4f}")
    table.add_row("mAP@50-95", f"{metrics.box.map:.4f}")
    table.add_row("Inference Latency", f"{metrics.speed['inference']:.2f} ms")
    table.add_row("Preprocess Latency", f"{metrics.speed['preprocess']:.2f} ms")
    table.add_row("Postprocess Latency", f"{metrics.speed['postprocess']:.2f} ms")

    console.print(table)

if __name__ == "__main__":
    main()
