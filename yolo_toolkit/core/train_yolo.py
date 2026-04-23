#!/usr/bin/env python3
"""
JetRacer Expert YOLO Training Engine (Rich-Enabled)
"""

import os
import yaml
import argparse
from ultralytics import YOLO
from pathlib import Path
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

console = Console()

def load_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def main():
    parser = argparse.ArgumentParser(description="JetRacer Expert Training Wrapper")
    parser.add_argument("--config", type=str, default="configs/hyperparameters.yaml", help="Path to hyperparams YAML")
    parser.add_argument("--data", type=str, default="dataset/data.yaml", help="Path to dataset YAML")
    parser.add_argument("--output", type=str, default="outputs", help="Output root")
    
    args = parser.parse_args()
    
    # 1. Load Expert Config
    if not os.path.exists(args.config):
        console.print(f"[bold red]Error: Config file {args.config} not found.[/bold red]")
        return
        
    cfg = load_config(args.config)
    
    # Display config Summary
    table = Table(title="Training Hyperparameters", title_style="bold cyan")
    table.add_column("Parameter", style="magenta")
    table.add_column("Value", style="green")
    for k, v in cfg.items():
        if isinstance(v, (int, float, str, bool)):
            table.add_row(k, str(v))
    
    console.print(table)

    if cfg.get('fliplr', 1.0) == 0.0:
        console.print(Panel("[bold yellow]SAFETY NOTICE[/bold yellow]: Horizontal Flip is [bold red]DISABLED[/bold red] to protect directional sign labels.", border_style="yellow"))

    # 2. Setup Directories
    Path(args.output).mkdir(parents=True, exist_ok=True)

    # 3. Initialize Model
    model = YOLO(cfg['model'])

    # 4. Run Training
    results = model.train(
        data=args.data,
        project=args.output,
        name="jetracer_hardened",
        **cfg 
    )

    console.print(Panel(f"[bold green]SUCCESS[/bold green]\nBest Weights: [cyan]{args.output}/jetracer_hardened/weights/best.pt[/cyan]", title="Training Complete", border_style="green"))

if __name__ == "__main__":
    main()
