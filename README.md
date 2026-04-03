# MowgliNext

Autonomous robot mower built on ROS2 Jazzy — a complete rewrite of OpenMower for YardForce Classic 500 hardware with LiDAR, behavior trees, and modern navigation.

**[Website](https://cedbossneo.github.io/mowglinext/)** | **[Wiki](https://github.com/cedbossneo/mowglinext/wiki)** | **[Discussions](https://github.com/cedbossneo/mowglinext/discussions)** | **[Issues](https://github.com/cedbossneo/mowglinext/issues)**

## Monorepo Structure

| Directory | Description |
|-----------|-------------|
| [`ros2/`](ros2/) | ROS2 stack: Nav2, SLAM Toolbox, behavior trees, coverage planner, hardware bridge |
| [`docker/`](docker/) | Docker Compose deployment, DDS config, service orchestration |
| [`sensors/`](sensors/) | Dockerized sensor drivers (GPS, LiDAR) — one directory per model |
| [`gui/`](gui/) | React + Go web interface for configuration, map editing, and monitoring |
| [`firmware/`](firmware/) | STM32 firmware for motor control, IMU, blade safety |

## Quick Start

```bash
git clone https://github.com/cedbossneo/mowglinext.git
cd mowglinext/docker
cp .env.example .env
nano config/mowgli/mowgli_robot.yaml  # Set your GPS datum, dock pose, NTRIP
docker compose up -d
```

GUI at `http://<mower-ip>:4006` | Foxglove at `ws://<mower-ip>:8765`

See the [Getting Started](https://github.com/cedbossneo/mowglinext/wiki/Getting-Started) wiki page for full setup instructions.

## Architecture

```
┌─────────────────────────────────────────────────┐
│  GUI (React + Go)          :4006                │
├─────────────────────────────────────────────────┤
│  ROS2 Stack (Jazzy)                             │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐│
│  │ Nav2     │ │ SLAM     │ │ Behavior Tree    ││
│  │ (navigate│ │ Toolbox  │ │ (main_tree.xml)  ││
│  │  dock    │ │          │ │                  ││
│  │  cover)  │ │          │ │                  ││
│  └──────────┘ └──────────┘ └──────────────────┘│
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐│
│  │ Coverage │ │ Localiz. │ │ Hardware Bridge  ││
│  │ Planner  │ │ (GPS+EKF)│ │ (serial ↔ ROS2) ││
│  │ (F2C v2) │ │          │ │                  ││
│  └──────────┘ └──────────┘ └──────────────────┘│
├──────────────────────┬──────────────────────────┤
│  Sensors (Docker)    │  STM32 Firmware          │
│  GPS (u-blox F9P)    │  Motor control           │
│  LiDAR (LD19)        │  IMU, blade safety       │
└──────────────────────┴──────────────────────────┘
```

## Documentation

| Resource | What's there |
|----------|-------------|
| [Website](https://cedbossneo.github.io/mowglinext/) | Landing page, features overview, getting started |
| [Wiki](https://github.com/cedbossneo/mowglinext/wiki) | Full reference: architecture, configuration, deployment, sensors, firmware, BT, FAQ |
| [Discussions](https://github.com/cedbossneo/mowglinext/discussions) | Community Q&A |

## Hardware

- YardForce Classic 500 chassis (500B, LUV1000Ri also supported)
- ARM64 SBC — Rockchip RK3566/RK3588, Raspberry Pi 4/5
- LDRobot LD19 LiDAR (2D, UART)
- u-blox ZED-F9P RTK GPS (USB-CDC)
- Custom STM32 board for motor/blade/IMU

## Contributing

We welcome contributions! Claude AI reviews every PR and assists in issues.

- [Contributing Guide](CONTRIBUTING.md)
- [Code of Conduct](CODE_OF_CONDUCT.md)
- Mention **@claude** in any issue or PR for AI assistance

## License

[GPLv3](LICENSE)
