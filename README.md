# rocket-gnc

A fully simulated rocket **GNC (Guidance, Navigation, Control) and FDIR (Fault Detection, Isolation and Recovery)** pipeline, built to explore embedded and aerospace software engineering without any physical hardware.

## Two projects in one

This repository contains **two distinct projects** that work together:

1. **Simulation** (Python): a rocket flight simulator that produces a "ground truth" flight, realistic noisy sensor data and a visualization.
2. **Flight software** (C): an independent program that receives only raw sensor measurements and must reconstruct the rocket's state (altitude, velocity, attitude), detect faults and, later, control the vehicle.

The two communicate **only through TCP sockets** using a documented binary protocol. As a consequence, **`flight_sw` is not tied to this simulator**: any program able to speak the protocol (another simulator, a game engine, a hardware test bench, a script replaying logs) can connect to it, send sensor data and receive state estimates. Likewise, the simulator could in principle be paired with a different flight software.

Each main folder has **its own README** with the detailed explanation of its content, build and usage. This top-level README gives the global picture.

> **Status:** early development. Phase 0 (repository setup) is complete.

---

## Table of contents

1. [Goals](#goals)
2. [Architecture](#architecture)
3. [Tech stack](#tech-stack)
4. [Repository layout](#repository-layout)
5. [Design principles](#design-principles)
6. [Roadmap](#roadmap)
7. [Planned versions](#planned-versions)
8. [Getting started](#getting-started)
9. [References](#references)

---

## Goals

- Learn and demonstrate the building blocks of a real flight software chain: sensor modelling, state estimation, fault detection and control.
- Keep the project close to real embedded practice: fixed-step loops, no dynamic allocation in flight code, binary protocols, strict separation between simulator and flight software.
- Provide clear visual feedback: ground truth vs raw sensor data vs estimator output.
- Make the flight software reusable on its own, through a clean TCP interface.

## Architecture

```
┌────────────────┐   truth    ┌────────────────┐  raw data   ┌────────────────┐
│   Simulator    │ ─────────► │ Sensor models  │ ──────────► │   flight_sw    │
│ (physics, 3D)  │            │ noise, bias,   │  TCP        │   (C program)  │
│     Python     │            │ rates, faults  │  lockstep   │ estimator, FDIR│
└───────┬────────┘            └────────────────┘             └───────┬────────┘
        ▲                                                            │
        │            commands (future: control, abort)               │
        └────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
                      ┌──────────────────────────┐
                      │      Visualization       │
                      │ truth / measures / estim.│
                      └──────────────────────────┘
```

**Data flow (lockstep):**

1. The simulator advances the physics by one fixed time step.
2. Sensor models turn the true state into degraded measurements (noise, bias, quantization, different sample rates).
3. The measurements are sent to `flight_sw` over TCP.
4. `flight_sw` runs its filter and replies with an estimate.
5. The visualization displays truth, raw measurements and estimate, then the next step begins.

The simulation is **synchronous**: Python controls simulated time, so runs are reproducible and independent of machine load. Simulated time comes from the timestamps in the packets, not from the wall clock.

## Tech stack

| Layer | Technology | Why |
|---|---|---|
| Simulation and physics | Python 3, NumPy | Fast to develop, vectorized math |
| Visualization | Matplotlib (RadioButtons for the 1D/2D/3D selector) | Simple plots, animation and widgets; Pygame or Rerun may come later |
| Flight software | C (C11), no external libraries | Same language as real embedded flight code; small matrices written by hand, no `malloc` |
| Communication | TCP sockets, binary little-endian protocol, `TCP_NODELAY` | Flight software runs as an independent process, like hardware-in-the-loop |
| Build (flight_sw) | Make or CMake, GCC/Clang (Winsock on Windows) | Decided when the C part starts |
| Version control | Git, single repository (monorepo) | Python and C evolve together; the protocol changes in one commit |

## Repository layout

```
rocket-gnc/
├── README.md          This file (global overview)
├── requirements.txt   Python dependencies
├── .gitignore
├── main.py            Entry point (planned): runs sim → sensors → flight_sw → viz
├── sim/               Simulator (Python)          → has its own README.md
├── viz/               Visualization (Python)      → has its own README.md
├── flight_sw/         Flight software (C)         → has its own README.md
└── docs/              Documentation               → has its own README.md
```

> **Each folder contains its own `README.md`** describing its purpose, structure, how to build or run it, and its public interface. Start here for the big picture, then go to the folder README for details.

### `sim/`
Everything that represents the "real world":
- Vector-based 3D physics (gravity as a function of altitude, thrust, drag, variable mass).
- Planet model (a scaled "toy" planet may be used so the rocket is visible on screen).
- Dimension constraints: 1D (radial motion only), 2D (planar) and 3D (free) are the same physics with different constraints.
- Sensor models: accelerometer, barometer, later gyroscope, magnetometer and GPS, each with its own rate, noise, bias and fault injection.

### `viz/`
Display only, no physics:
- Planet and rocket as colored dots with trails.
- White dot = true position, red dot = position estimated by `flight_sw`.
- 1D / 2D / 3D mode selector.
- Plots of altitude, velocity and estimation error (estimate minus truth), plus raw sensor data.

### `flight_sw/`
The flight software, built as a **standalone C program** and designed to be **reusable outside this project**: it exposes a TCP interface, so any client following `docs/PROTOCOL.md` can feed it sensor data and read its estimates.
- 1D Kalman filter fusing accelerometer (prediction) and barometer (correction).
- TCP server receiving measurement packets and returning estimates.
- Later: 3D EKF, FDIR, attitude control and flight-mode supervisor.
- Hard rule: **it never reads the ground truth**, only sensor data.

### `docs/`
- `PROTOCOL.md`: binary TCP protocol between the simulator and `flight_sw` (message layouts, versioning). This is the main contract between the two sides, and the only thing a third-party client needs to use `flight_sw`.
- Future: architecture notes, sensor parameters, validation results.

## Design principles

- **Two independent projects:** simulation and flight software only share the TCP protocol, never code or memory.
- **Truth isolation:** `flight_sw` never receives the true position, velocity or attitude. The only data sent to it are raw sensor measurements (accelerometer, barometer, and later gyroscope, magnetometer and GPS), exactly as it would receive them on a real rocket. Everything else (altitude, velocity, attitude) must be computed by the flight software itself (can not be determine from any sensors in real-life).
- **Physics written in 3D from day one:** 1D and 2D are constrained 3D, so there is one simulator, not three.
- **Imperfect sensors by design:** real sensors never return exact values. They produce noisy, biased, quantized and delayed measurements, so the simulator deliberately degrades the ground truth in the same way. The flight software must therefore work with imprecise data and estimate the true state despite it. Without this noise there would be nothing to estimate, and no way to test the filter or the fault detection.

- **Multi-rate fusion:** the accelerometer runs fast (e.g. 1 kHz), the barometer slower (e.g. 50 Hz), GPS slower still. The filter predicts every step and corrects only when a new measurement arrives.
- **Embedded-style C:** fixed-size buffers, no dynamic allocation, fixed-rate loop, explicit fixed-width types.
- **Explicit protocol:** fixed-size binary messages, little-endian, versioned. It is the contract that makes `flight_sw` usable with any other client.
- **One README per folder:** each component documents itself.

## Roadmap

### Phase 0: Repository setup
- [x] Monorepo structure, README, protocol draft
- [ ] One README per folder (`sim/`, `viz/`, `flight_sw/`, `docs/`)

### Phase 1: Simulator and visualization (Python)
- [ ] Vector physics: gravity, thrust, drag, planet
- [ ] Fixed-step integration (semi-implicit Euler, then RK4)
- [ ] Animation: planet, rocket dot, trail
- [ ] 1D / 2D / 3D mode selector
- [ ] Validation against the analytical apogee formula

### Phase 2: Sensor models
- [ ] Accelerometer (white noise, bias, quantization, specific force)
- [ ] Barometer (noise, latency, lower rate, atmosphere model)
- [ ] Display raw measurements next to the truth

### Phase 3: Flight software, 1D
- [ ] Linear Kalman filter in C (altitude, velocity)
- [ ] Python ↔ C link over TCP (lockstep)
- [ ] White dot (truth) vs red dot (estimate), error plots

### Phase 4: 3D navigation
- [ ] Gyroscope and attitude estimation (complementary / Madgwick / Mahony)
- [ ] Quaternion-based EKF (position, velocity, attitude, sensor biases)
- [ ] Magnetometer and GPS

### Phase 5: FDIR
- [ ] Fault injection (stuck gyro, sudden bias, saturated barometer, sensor loss)
- [ ] Innovation monitoring (chi-squared / Mahalanobis tests) and consistency checks
- [ ] Software redundancy (e.g. two simulated IMUs with voting)

### Phase 6: Control and safety supervisor
- [ ] Attitude controller (PID, then LQR) with simulated thrust vector control
- [ ] Flight state machine: pad → boost → coast → apogee → descent → parachute
- [ ] Safe modes: engine cut-off, parachute deployment, degraded mode

### Phase 7: Realism and polish
- [ ] Engine vibrations, sensor temperature drift
- [ ] Log recording and replay
- [ ] Richer 3D rendering
- [ ] Optional: port `flight_sw` to a microcontroller (hardware-in-the-loop)

## Planned versions

| Version | Content |
|---|---|
| **v0.1** | Simulator and animation with 1D/2D/3D selector, truth only |
| **v0.2** | Accelerometer and barometer models, raw data display |
| **v0.3** | C flight software with 1D Kalman filter, TCP link, truth vs estimate comparison |
| **v0.4** | 3D navigation: gyroscope, attitude EKF, magnetometer, GPS |
| **v0.5** | FDIR: fault injection and detection |
| **v0.6** | Closed loop: attitude control and safety supervisor |
| **v1.0** | Complete, documented, reproducible pipeline with validation results |

Versions are indicative and may be reordered as the project evolves.

## Getting started

```bash
git clone <repository-url>
cd rocket-gnc

python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Running instructions will be added as soon as `main.py` exists. Build instructions for `flight_sw` will be added with Phase 3.

## References

- Roger Labbe, *Kalman and Bayesian Filters in Python* (free online)
- Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*
- PX4 and ArduPilot (EKF2, failure detection) as open-source references
- Public sensor datasheets (e.g. MPU-6050, BMP280) for realistic noise parameters
