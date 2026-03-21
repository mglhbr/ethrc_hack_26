# ETHRC 2026 - WBC Commanding Interface Testbed (Unitree G1)

This repository is a lightweight testbed for experimenting with commanding interfaces for **Whole-Body Control (WBC)** during the **ETHRC 2026 hackathon**, targeting the **Unitree G1 humanoid** platform.

## Purpose

The goal is to rapidly prototype and evaluate different ways to command robot behavior at multiple levels, including:

- high-level task commands (intent-level)
- motion or posture commands (mid-level)
- low-level references/constraints for WBC loops

This repo is intended for fast iteration, interface experiments, and integration checks rather than production deployment.

## Scope

Use this project to:

- define command schemas and APIs
- test command translation pipelines
- validate command timing/latency assumptions
- compare interface ergonomics for teleop, scripted control, and autonomy hooks

## Suggested Architecture

As you implement, organize code around clear boundaries:

- `interfaces/`: command definitions, message schemas, protocol adapters
- `controllers/`: WBC-facing bridges and conversion logic
- `sim/` or `hw/`: simulation/hardware-specific runners
- `tests/`: unit and integration tests for command handling
- `scripts/`: local run/debug utilities

## Getting Started

1. Clone the repository.
2. Create a branch for your experiment:
   ```bash
   git checkout -b feat/<short-topic>
   ```
3. Add your first interface prototype and a minimal runnable example.
4. Document assumptions (robot mode, control frequency, safety constraints).

## Development Notes

- Keep interfaces versioned as they evolve.
- Prefer explicit units in all command payloads (e.g., SI units).
- Treat safety checks (joint limits, contact assumptions, emergency stop pathways) as first-class requirements.
- Record observed behavior and timing metrics with each experiment.

## Collaboration

When contributing:

- keep PRs small and focused
- include a short experiment report in the PR description
- attach logs/plots if interface behavior changed

## License

Add a license before public distribution.
