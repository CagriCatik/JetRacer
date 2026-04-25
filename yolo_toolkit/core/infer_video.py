#!/usr/bin/env python3
import cv2
import os
import glob
from ultralytics import YOLO
from rich.console import Console

console = Console()

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Generate a video from dataset images and run YOLO inference.")
    parser.add_argument("--weights", type=str, required=True, help="Path to trained weights (e.g. best.pt)")
    parser.add_argument("--source", type=str, required=True, help="Path to folder containing dataset images")
    parser.add_argument("--output", type=str, default="outputs/inference_demo.mp4", help="Output annotated video path")
    parser.add_argument("--fps", type=int, default=1, help="FPS for the generated video (lower = slower playback)")
    args = parser.parse_args()

    # Find images in source directory
    image_paths = sorted(glob.glob(os.path.join(args.source, "*.*")))
    image_paths = [p for p in image_paths if p.lower().endswith(('.png', '.jpg', '.jpeg'))]

    if not image_paths:
        console.print(f"[bold red]No images found in {args.source}![/bold red]")
        return

    console.print(f"[bold green]Found {len(image_paths)} images. Compiling video and running inference...[/bold green]")

    # Load Model
    console.print(f"[bold cyan]Loading YOLO model from: {args.weights}[/bold cyan]")
    model = YOLO(args.weights)

    # Initialize Video Writer
    first_frame = cv2.imread(image_paths[0])
    if first_frame is None:
        console.print("[bold red]Failed to read the first image.[/bold red]")
        return
        
    h, w, _ = first_frame.shape
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_video = cv2.VideoWriter(args.output, fourcc, args.fps, (w, h))

    console.print("[bold yellow]Starting inference loop. Press 'q' in the video window to stop early.[/bold yellow]")

    for i, img_path in enumerate(image_paths):
        frame = cv2.imread(img_path)
        if frame is None:
            continue
            
        # Run inference
        results = model.predict(frame, conf=0.5, verbose=False)
        
        # Plot detections on the frame
        annotated_frame = results[0].plot()
        
        # Write the annotated frame to the video
        out_video.write(annotated_frame)
        
        # Show real-time progress
        cv2.imshow("Real Inference Validation", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            console.print("[bold red]Inference interrupted by user.[/bold red]")
            break
            
        # Console progress
        if i % 20 == 0:
            console.print(f"Processed {i}/{len(image_paths)} frames...")

    out_video.release()
    cv2.destroyAllWindows()
    console.print(f"[bold green]Validation complete! Annotated video saved to: {args.output}[/bold green]")

if __name__ == "__main__":
    main()

# python main.py infer_video --weights outputs/jetracer_hardened/weights/best.pt --source ../dataset/train/images