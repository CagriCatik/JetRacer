#!/usr/bin/env python3
"""
JetRacer Dataset Intelligence Tool
Analyzes class distribution and label integrity.
"""

import os
import yaml
import argparse
from pathlib import Path
from rich.console import Console
from rich.table import Table
from rich.progress import track
from collections import Counter

console = Console()

def main():
    parser = argparse.ArgumentParser(description="JetRacer Dataset Analyzer")
    parser.add_argument("--data", type=str, default="dataset/data.yaml", help="Path to data.yaml")
    args = parser.parse_args()

    # Normalize the data path relative to the toolkit root
    data_path = Path(args.data).absolute()
    if not data_path.exists():
        # Try relative to CWD
        data_path = (Path.cwd() / args.data).absolute()

    if not data_path.exists():
        console.print(f"[bold red]Error: {data_path} not found.[/bold red]")
        return

    with open(data_path, 'r') as f:
        data_cfg = yaml.safe_load(f)

    classes = data_cfg['names']
    
    # Correct path resolution relative to where data.yaml is located
    # In YOLO, images and labels are siblings
    data_dir = data_path.parent
    train_relative = data_cfg['train']
    
    # Resolve the absolute path of the training images
    train_img_path = (data_dir / train_relative).resolve()
    
    # Path to labels (replace 'images' with 'labels' in the folder name)
    label_path = Path(str(train_img_path).replace('images', 'labels'))
    
    console.print(f"[bold blue]Analyzing training split at:[/bold blue] [cyan]{label_path}[/cyan]")
    
    if not label_path.exists():
        console.print(f"[bold red]Error: Label directory not found at {label_path}[/bold red]")
        return

    label_files = list(label_path.glob('*.txt'))
    class_counts = Counter()
    total_boxes = 0

    for lb in track(label_files, description="Scanning Labels..."):
        with open(lb, 'r') as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if parts:
                    class_id = int(parts[0])
                    class_counts[class_id] += 1
                    total_boxes += 1

    # Status Table
    table = Table(title="Dataset Distribution (Training Set)", title_style="bold magenta")
    table.add_column("Class ID", style="dim")
    table.add_column("Class Name", style="cyan")
    table.add_column("Count", style="green", justify="right")
    table.add_column("Percentage", style="yellow", justify="right")

    for i, name in enumerate(classes):
        cnt = class_counts[i]
        pct = (cnt / total_boxes * 100) if total_boxes > 0 else 0
        table.add_row(str(i), name, str(cnt), f"{pct:.1f}%")

    console.print(table)
    console.print(f"\n[bold]Total Annotations:[/bold] [cyan]{total_boxes}[/cyan]")
    
    # Expert Advice
    if total_boxes > 0:
        if any(count < (total_boxes / len(classes) * 0.2) for count in class_counts.values()):
            console.print("\n[bold yellow]ADVICE[/bold yellow]: Significant class imbalance detected. Consider adding more images for rare classes or using oversampling.")
    else:
        console.print("\n[bold red]WARNING[/bold red]: No annotations found. Check your dataset folder structure.")

if __name__ == "__main__":
    main()
