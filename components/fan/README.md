# Thermostatic CPU Fan Controller

This directory maintains the bash and systemd daemon infrastructure for the active cooling unit mounted directly onto the Jetson Nano Maxwell heatsink.

## Architecture
The Jetson Nano CPU and GPU complexes run exceptionally hot when simultaneously rendering PyTorch Tensor frames and 2D SLAM occupancies. If the core temperature exceeds `85°C`, the Jetpack OS aggressively thermal-throttles the CPU clocks, completely shattering our latency requirements for `twist_mux` priority arbitration.

Therefore, we mount a controllable PWM fan to the chassis. By deploying this systemd daemon onto the **Host OS**, it automatically ramps PWM speeds via GPIO mappings based on active thermal signatures polled out of `/sys/class/thermal/`.

## Deployment

These scripts **must be executed on the Host OS**, not inside the Docker Container! Host temperatures cannot be accurately read nor fan speeds modulated from within an isolated virtualization layer safely.

To install the thermostatic daemon as an auto-starting service:
```bash
sudo ./jetson-fan-control-install.sh
```

To remove the service and release the GPIO locks:
```bash
sudo ./jetson-fan-control-uninstall.sh
```

### Tuning Constraints
The cooling curves are defined inside the `configs/` folder. By default, the fan lies dormant under `35°C` to reduce audible annoyance, spins up softly towards `45°C`, and hits Maximum RPM at `60°C` to definitively prevent any thermal runaway.
