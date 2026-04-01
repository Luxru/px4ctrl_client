<a id="readme-top"></a>
<br />
<div align="center">
  <a href="https://github.com/Luxru/px4ctrl_client">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>
  <h3 align="center">UISC Lab Px4Ctrl Client</h3>
  <p align="center">Zenoh + ImGui Ground Control Station for px4ctrl</p>
  <img src="https://img.shields.io/badge/license-GPL--3.0-blue" alt="license"/>
</div>

## About
`px4ctrl_client` is a lightweight desktop GCS for `px4ctrl`.


## Features
- Mission commands: `ARM`, `ENTER_OFFBOARD`, `TAKEOFF`, `LAND`, `FORCE_HOVER`, `FORCE_DISARM`, `ALLOW_CMD_CTRL`.
- Hover target editing and publish (`CHANGE_HOVER_POS`).
- Online safety update (`SET_SAFETY_LIMITS`).
- Status panel with highlighted `Offboard` and `Armed` states.
- XY position trace with optional geofence box overlay.
- X/Y/Z and control command time-series plots with axis ticks and second-based time axis.
- Keyboard piloting with configurable velocity.
- Colored logs by log level (`trace/debug/info/warn/error/critical`).

## Prerequisites
- C++20 compiler
- `spdlog`
- `OpenGL`
- `glfw3`
- `pkg-config`
- `zenoh-c`

## Build
```bash
git clone https://github.com/Luxru/px4ctrl_client.git
cd px4ctrl_client
git submodule update --init --recursive
mkdir -p build
cd build
cmake ..
make -j4
```

## Run
```bash
./px4client -c ../config/zenoh.json
```

Default config file:
- `config/zenoh.json`

## Transport Config (JSON)
Example:

```json
{
  "backend": "zenoh",
  "server_topic": "px4s",
  "client_topic": "px4c",
  "log_topic": "px4log",
  "zenoh": {
    "mode": "peer",
    "connect": "",
    "listen": "",
    "multicast_scouting": true,
    "scouting_timeout_ms": 1000
  },
  "keyboard": {
    "vel_xy": 1.0,
    "vel_z": 0.2,
    "vel_yaw": 2.0
  }
}
```

Notes:
- `listen` default is empty to reduce local port conflicts.
- `connect` can stay empty when scouting is enabled in the same network.
- `telemetry_hz` can be configured in the same JSON (default: `200`).

## Keyboard Control
Keyboard listener is per drone card and must be activated from the UI.

Keys:
- `W/S/A/D`: move hover target in XY.
- `R/F`: move up/down.
- `Q/E`: yaw increase/decrease.
- `C`: toggle BODY/WORLD control frame.
- `Space`: send `FORCE_HOVER`.
- `J`: send `FORCE_DISARM`.
- `L`: send `LAND`.

Velocity sources:
- Initial values from `config/zenoh.json -> keyboard`.
- Runtime adjustment from ImGui panel.

## Safety Online Config
Safety panel supports:
- `Enable Geofence`
- `Geofence Min/Max`
- `Enable Attitude Fence`
- `Max Roll Deg`, `Max Pitch Deg`, `Max Yaw Deg`

Limit rule:
- `-1` means unlimited.
- Otherwise each limit must be in `(0, 180]`.

## RC Gate Behavior
When server-side `guard.use_rc = true`:
- Client will still show telemetry/logs normally.
- ARM/DISARM/OFFBOARD related commands can be rejected by server policy.
- Server expects RC/PX4 mode switching for those operations.

## Troubleshooting
- `Failed to open zenoh session`:
  - Check `zenohc` installation and runtime library path.
  - Try `listen: ""` and keep only `connect` or scouting.
- No telemetry displayed:
  - Verify topic names and that server/client use the same transport JSON fields.
- Unexpected decode/ABI issues:
  - Ensure client and server are built from matching message struct versions.

## Screenshot
![image](images/image.png)

## Contact
Xu Lu - lux@cqu.edu.cn

## Acknowledgments
- [ZJU FastLab](https://github.com/ZJU-FAST-Lab)
- [UZH Robotics and Perception Group](https://github.com/uzh-rpg)
