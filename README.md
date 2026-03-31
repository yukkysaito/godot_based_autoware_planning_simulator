# Godot Vehicle Simulator for Autoware

A Godot 4-based vehicle dynamics simulator designed as a replacement for Autoware's `simple_planning_simulator`. Features realistic physics, lanelet2 map rendering, and full Autoware interface compatibility via rosbridge.

![Demo](docs/image/Godot_based_planning_simulator.gif)

## Features

- **Vehicle Physics**: VehicleBody3D-based simulation with configurable parameters (mass, wheelbase, suspension, tire grip, etc.)
- **Autoware Integration**: Drop-in replacement for `simple_planning_simulator` — subscribes to control commands and publishes vehicle status via rosbridge
- **Lanelet2 Map Rendering**: Dynamically loads lanelet2 maps from `/map/vector_map`, rendering road surfaces, intersection areas, hatched road markings, parking lots, road borders (walls), shoulder areas, and road markings
- **Viewer Frame Support**: Uses `map → viewer` TF to avoid floating-point precision issues with large map coordinates
- **External Vehicle Params (JSON)**: Load/save vehicle parameters from external JSON files — editable without rebuilding
- **In-Game Tuning Panel**: All vehicle parameters adjustable in real-time (Tab key) with Import/Export JSON support
- **Telemetry Graphs**: Real-time lateral/longitudinal G-force, jerk, and control input visualization
- **Gear System**: P/R/N/D with creep simulation
- **Transport Delay + 1st Order Lag**: Configurable control response delays for realistic actuator simulation
- **Sensor Output Delay**: Configurable per-topic output delay for simulating sensor latency
- **Manual/Autonomous Switching**: M key or Autoware engage topic

## Requirements

- [Godot Engine 4.2+](https://godotengine.org/) (tested with [4.2.1-stable](https://github.com/godotengine/godot/tree/b09f793f564a6c95dc76acc654b390e68441bd01))
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) (ROS 2)
- [Autoware](https://github.com/autowarefoundation/autoware) (for lanelet2 maps and control commands)
- Python 3 with `lanelet2` package (for the bridge node)

## Project Structure

```
driving_game/
├── project.godot              # Godot project config
├── main.tscn / main.gd       # Main scene and setup
├── car.tscn / car.gd         # Vehicle controller
├── vehicle_params.json        # Default vehicle parameters (external JSON)
├── follow_camera.gd           # Orbit camera (RMB + drag)
├── ros_bridge.gd              # Autoware ↔ Godot WebSocket bridge
├── lanelet_map.gd             # Lanelet2 map mesh builder
├── tuning_panel.gd            # In-game parameter tuning UI
├── telemetry_graph.gd         # G-force / jerk graphs
├── control_telemetry.gd       # Throttle / brake / steering graphs
├── trail_renderer.gd          # Vehicle trajectory trail
└── scripts/
    └── lanelet_bridge_node.py # ROS 2 node: lanelet2 → Godot bridge
```

## Quick Start

### 1. Download and extract

Download the latest release from [Releases](https://github.com/yukkysaito/godot_based_autoware_planning_simulator/releases):

```bash
tar xzf godot_autoware_simulator-v*.tar.gz
```

This extracts the following files:

```
godot_autoware_simulator.x86_64   # Simulator binary
vehicle_params.json                # Vehicle parameters (editable)
scripts/lanelet_bridge_node.py     # ROS 2 lanelet2 bridge
```

### 2. Start Autoware (with planning simulator disabled)

Before launching Autoware, comment out the existing `simple_planning_simulator` in your simulator launch file.
In [`tier4_simulator_launch/launch/simulator.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/tier4_universe_launch/tier4_simulator_launch/launch/simulator.launch.xml#L273-L279), comment out:

```xml
<!-- Comment out the following block -->
<!--
<group>
  <push-ros-namespace namespace="simulator"/>
  <include file="$(find-pkg-share autoware_simple_planning_simulator)/launch/simple_planning_simulator.launch.py">
    ...
  </include>
</group>
-->
```

Then launch Autoware as usual. rosbridge_server should be included in the launch.
If it is not running, start it manually:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml max_message_size:=50000000
```

### 3. Start lanelet bridge

This node converts the lanelet2 binary map to geometry data for Godot.
It needs to be started in an environment where Autoware packages are sourced.
The bridge and the simulator can be started in any order.

```bash
source /path/to/autoware/install/setup.bash
python3 scripts/lanelet_bridge_node.py
```

The bridge stays running and serves map data on demand. You do **not** need to restart it when restarting the simulator.

### 4. Start the simulator

```bash
./godot_autoware_simulator.x86_64
```

The simulator automatically connects to rosbridge (`ws://localhost:9090`) and requests the lanelet map.

**With a custom vehicle params file:**

```bash
./godot_autoware_simulator.x86_64 --vehicle-params /path/to/custom_params.json
```

**From source (Godot editor):**

```bash
godot --path /path/to/driving_game
```

Or open the project in the Godot editor and press F5.

### 4. Set initial pose

Use RViz2's "2D Pose Estimate" tool to place the vehicle on the map.
Until an initial pose is set, the vehicle starts on a default 1km x 1km ground plane at the origin.

## Fit Vehicle Params From A Recorded MCAP

Use [`scripts/fit_vehicle_params.py`](scripts/fit_vehicle_params.py) to estimate a first-pass parameter update from a recorded bag.

```bash
source /opt/ros/humble/setup.bash
source /path/to/autoware/install/setup.bash
python3 scripts/fit_vehicle_params.py /path/to/log_dir \
  --output-report /tmp/driving_game_fit_report.json \
  --write-params /tmp/driving_game_fit_params.json
```

The fitter currently targets the parameters that are identifiable from control, odometry, acceleration, and steering logs:

- response delays / time constants for accel, brake, steering
- understeer gradient
- heuristic actuation / brake scaling

Resistance parameters are reported conservatively and may be left unchanged when the bag does not contain enough clean coast data.

## Controls

| Key | Action |
|-----|--------|
| W / Up | Accelerate |
| S / Down | Brake |
| A / Left | Steer left |
| D / Right | Steer right |
| 1 / 2 / 3 / 4 | Gear: P / R / N / D |
| R | Respawn (nearby road) |
| T | Respawn (initial position) |
| M | Toggle Manual / Autonomous |
| Tab | Open/Close tuning panel |
| RMB + Drag | Orbit camera |
| Scroll | Zoom in/out |
| Esc | Quit |

## Autoware Interface

### Input Topics (Subscribe)

| Topic | Type | Description |
|-------|------|-------------|
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | Steering + acceleration command |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | Gear command |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | Turn signal |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard lights |
| `/initialpose3d` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial pose |
| `/vehicle/engage` | `autoware_vehicle_msgs/msg/Engage` | Engage autonomous mode |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | For `map → viewer` transform |

### Output Topics (Publish)

| Topic | Type | Description |
|-------|------|-------------|
| `/tf` | `tf2_msgs/msg/TFMessage` | `map → base_link` transform |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | Vehicle odometry |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | Vehicle acceleration |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | Velocity report |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | Steering angle report |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | Gear report |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | Control mode (Manual/Auto) |
| `/vehicle/status/turn_indicators_status` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | Turn signal status |
| `/vehicle/status/hazard_lights_status` | `autoware_vehicle_msgs/msg/HazardLightsReport` | Hazard light status |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/control/control_mode_request` | `autoware_vehicle_msgs/srv/ControlModeCommand` | Switch Auto/Manual |

## Lanelet Bridge

The `lanelet_bridge_node.py` subscribes to `/map/vector_map` (LaneletMapBin), deserializes the lanelet2 map, and serves geometry data to Godot via rosbridge services. Godot automatically requests the map data on startup.

### Bridge Services

| Service | Type | Description |
|---------|------|-------------|
| `/godot/lanelet_batch_count` | `std_srvs/srv/Trigger` | Returns number of geometry batches |
| `/godot/lanelet_batch` | `example_interfaces/srv/SetBool` | Returns next batch (data=true to reset) |

## Vehicle Parameters

Vehicle parameters are stored in `vehicle_params.json`. On startup, the simulator loads parameters in the following priority:

1. **CLI argument**: `--vehicle-params /path/to/file.json`
2. **Next to the binary**: `vehicle_params.json` in the same directory as the executable
3. **Embedded fallback**: Built-in default values inside the binary

### Editing Parameters

**Option A — Edit the JSON file directly:**

Edit `vehicle_params.json` next to the binary and restart the simulator. The default file is sectioned into `common`, `input_source`, `control_cmd`, `actuation_cmd`, and `sensor_delay`.

**Option B — Use the in-game tuning panel (Tab key):**

Adjust sliders in real-time, then click **Export JSON** to save to a file.
Use **Import JSON** to load a different parameter file at runtime.
**Reset Defaults** reloads values from the currently loaded JSON.

### Parameter Categories

- **Geometry**: Wheelbase, tread, overhang, wheel radius (requires car rebuild)
- **Powertrain**: Engine force, brake force, steering angle, creep
- **Control Cmd**: Delay, time constant, and brake mapping used when `control_cmd` drives the sim
- **Actuation Cmd**: Delay, time constant, and fallback scaling used when `actuation_cmd` drives the sim
- **Sensor Delay**: Output delay for TF, odometry, velocity, steering, acceleration
- **Resistance**: Rolling resistance, drag coefficient, frontal area
- **Suspension**: Stiffness, travel, damping, max force, friction slip
- **Lateral Dynamics**: Understeer gradient, drivetrain efficiency
- **Vehicle**: Weight, center of mass

## Web Build (Browser)

The simulator can also run in a web browser. See [docs/web_build.md](docs/web_build.md) for instructions.

## License

Apache License 2.0
