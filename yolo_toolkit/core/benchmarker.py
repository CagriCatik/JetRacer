#!/usr/bin/env python3
"""
JetRacer Hardware Benchmarker
Calculates FPS and Jitter for .pt or .engine models.
"""

import argparse
import time
import torch
import numpy as np
from ultralytics import YOLO
from rich.console import Console
from rich.table import Table

console = Console()

def main():
    parser = argparse.ArgumentParser(description="JetRacer Hardware Benchmarker")
    parser.add_argument("--weights", type=str, required=True, help="Model to benchmark")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size")
    parser.add_argument("--frames", type=int, default=100, help="Number of frames to test")
    
    args = parser.parse_args()

    console.print(f"[bold blue]Warming up hardware with:[/bold blue] [cyan]{args.weights}[/cyan]")
    model = YOLO(args.weights)
    
    # Create dummy frame
    dummy_frame = np.random.randint(0, 255, (args.imgsz, args.imgsz, 3), dtype=np.uint8)

    # Warmup
    for _ in range(10):
        model.predict(dummy_frame, verbose=False)

    console.print(f"[bold green]Starting benchmark loop ({args.frames} iterations)...[/bold green]")
    
    latencies = []
    
    start_time = time.perf_counter()
    for _ in range(args.frames):
        iter_start = time.perf_counter()
        model.predict(dummy_frame, verbose=False)
        latencies.append((time.perf_counter() - iter_start) * 1000) # ms
    
    total_time = time.perf_counter() - start_time
    fps = args.frames / total_time
    
    # Stats Table
    table = Table(title="Hardware Performance Report", title_style="bold magenta")
    table.add_column("Metric", style="cyan")
    table.add_column("Value", style="green", justify="right")

    table.add_row("Average FPS", f"{fps:.2f} Hz")
    table.add_row("Mean Latency", f"{np.mean(latencies):.2f} ms")
    table.add_row("95th Percentile", f"{np.percentile(latencies, 95):.2f} ms")
    table.add_row("Max Jitter", f"{(np.max(latencies) - np.min(latencies)):.2f} ms")

    console.print(table)
    
    if fps < 10:
        console.print("[bold red]WARNING[/bold red]: Framerate is below 10Hz. This will cause significant reactive delay in the control loop!")

if __name__ == "__main__":
    main()
