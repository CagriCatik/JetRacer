# 3.5mm Analog Audio Telemetry (Speaker)

This directory isolates the ALSA and PortAudio outputs pushing to the onboard amplifier and speaker elements.

## Architecture
The JetRacer utilizes an external USB DAC or an embedded I2S amplifier chip to broadcast acoustic arrays. Because the robotic platform pushes audio natively while isolated inside a Docker boundary architecture, mapping the host's `/dev/snd` system is critical.

This system is utilized strictly by the Voice Commander (`jetracer_voice`) node to acknowledge offline TTS (Text-to-Speech) feedback sequences, confirming physical waypoint targeting!

## Diagnostic Usage
You can violently test the ALSA system using the onboard diagnostic script:
```bash
python3 jetson_speaker_test.py
```
This forces the native ALSA `speaker-test` subsystem to bypass Python rendering and speak pure WAV audio mapping to the hardware endpoints.

---
> [!CAUTION]
> If your ALSA drivers fail here, `twist_mux` and YOLO will continue operating properly, but your offline Vosk Voice Commander will permanently crash on instantiation!
