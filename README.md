# Mowgli Docker — v3 (ROS2 Jazzy)

v3 is a complete rewrite of the container deployment.  The ROS1 stack
(roscore, rosserial, openmower) has been replaced by a single
`mowgli_ros2` container running ROS2 Jazzy.

## What changed from v2

| v2 (ROS1 Noetic) | v3 (ROS2 Jazzy) |
|------------------|-----------------|
| roscore | removed — DDS has no master |
| rosserial | removed — hardware bridge is inside mowgli_ros2 |
| openmower | removed — replaced by mowgli_ros2 (Nav2 + BT + coverage) |
| foxglove-bridge (separate image) | built into mowgli_ros2, launched optionally |
| rosbridge (separate container) | built into mowgli_ros2, enabled via launch arg |

## Services

| Service | Description | Port(s) |
|---------|-------------|---------|
| `mowgli` | ROS2 hardware bridge, Nav2, behavior trees, SLAM, rosbridge, foxglove | 9090 (rosbridge), 8765 (foxglove) |
| `gui` | OpenMower GUI — Go backend + React frontend | host networking |
| `mosquitto` | MQTT broker for Home Assistant and telemetry | 1883, 9001 |
| `watchtower` | Automatic image update polling (gui only) | — |
| `web` | Static nginx landing page | 4005 |

## Quick start (local, direct serial)

### 1. Install Docker

```bash
curl https://get.docker.com | sh
```

### 2. udev rules (so devices get stable symlinks)

Create `/etc/udev/rules.d/50-mowgli.rules`:

```
# Mowgli STM32 board
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli"
# simpleRTK2B
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps"
# RTK1010Board (ESP USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps"
```

Reload: `sudo udevadm control --reload && sudo udevadm trigger`

### 3. Configure

Edit `.env` — in most cases only `MOWER_IP` needs to be set for ser2net mode.
The image tags default to `:v3`.

If your board appears on a path other than `/dev/ttyUSB0`, create a
`docker-compose.override.yaml`:

```yaml
services:
  mowgli:
    devices:
      - /dev/mowgli:/dev/ttyUSB0
```

### 4. Launch

```bash
docker compose up -d
```

The GUI is available on `http://<pi-ip>:4005` (landing page) once the stack is up.

## Configuration

### GUI settings — `config/om/`

Place `mower_config.sh` and any GUI-specific config files here.  This directory
is bind-mounted read-only at `/config` inside the `gui` container.

### ROS2 parameters — `config/mowgli/`

Place YAML parameter override files here.  The directory is bind-mounted
read-only at `/ros2_ws/config/` inside the `mowgli` container.

See [`config/mowgli/README.md`](config/mowgli/README.md) for the full list of
overrideable files and an example.

### MQTT — `config/mqtt/mosquitto.conf`

Standard Mosquitto configuration file.

## Foxglove Studio

Foxglove Bridge runs inside the main `mowgli` container on port **8765**.
Connect Foxglove Studio to `ws://<pi-ip>:8765`.

To run Foxglove Bridge as a separate container (useful if you want to restart
it independently):

```bash
docker compose -f docker-compose.yaml -f docker-compose.foxglove.yaml up -d
```

When using this override, set `enable_foxglove:=false` in the mowgli service
command inside a `docker-compose.override.yaml` to avoid the port conflict.

## Deployment modes

### Ser2net — remote brain, serial over TCP

Use when the compute board (Pi running Docker) is separate from the mower
board, connected via Ethernet.

On the mower Pi, install and configure `ser2net` to expose `/dev/mowgli` on
TCP port 4001 and `/dev/gps` on TCP port 4002.  Then on the brain machine:

```bash
# Edit .env: set MOWER_IP to the mower Pi's IP
docker compose -f docker-compose.ser2net.yaml up -d
```

### Remote split — nav on a powerful host, hardware on the Pi

Run Nav2, behavior trees, the GUI, and the map server on a desktop/server.
Run only the hardware bridge on the mower Pi.

On the mower Pi (serial access required):

```bash
docker compose -f docker-compose.remote.pi.yaml up -d
```

On the remote host:

```bash
docker compose -f docker-compose.remote.host.yaml up -d
```

Both machines must share the same `ROS_DOMAIN_ID` in `.env`.  DDS multicast
must be routable between them (same L2 segment), or configure FastDDS unicast
peer discovery.

## Logs

```bash
# Follow mowgli ROS2 output
docker compose logs -f mowgli

# Follow GUI
docker compose logs -f gui
```

## Updating

Watchtower checks for a new `gui` image every 4 hours.  To update the
`mowgli` image manually:

```bash
docker compose pull mowgli
docker compose up -d mowgli
```

## Shutdown

```bash
docker compose down
```

SLAM maps are preserved in the `mowgli_maps` Docker volume across restarts.
