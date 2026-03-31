extends Node
## Autoware-compatible vehicle simulator bridge via rosbridge_suite WebSocket.
## Subscribes to Autoware control commands and publishes vehicle status.

var car: VehicleBody3D
var lanelet_map: Node  # lanelet_map.gd instance — set by main.gd
var traffic_light_manager: Node3D  # traffic_light_manager.gd instance — set by main.gd
var trajectory_mesh: MeshInstance3D  # trajectory_mesh.gd instance — set by main.gd
var dynamic_object_mesh: MeshInstance3D  # dynamic_object_mesh.gd instance — set by main.gd

@export var rosbridge_url: String = "ws://localhost:9090"
@export var publish_rate_hz: float = 30.0
## Viewer frame offset: map -> viewer translation (set from /tf).
## All internal Godot coordinates are in the viewer frame (near zero).
## When publishing to ROS, this offset is added back to get map frame coords.
var viewer_offset_x: float = 0.0  # ROS X (east)
var viewer_offset_y: float = 0.0  # ROS Y (north)
var viewer_offset_z: float = 0.0  # ROS Z (up)
var viewer_offset_valid: bool = false

const INPUT_SOURCE_NONE = -1
const INPUT_SOURCE_CONTROL_CMD = 0
const INPUT_SOURCE_ACTUATION_CMD = 1

const CONTROL_CMD_PARAM_PROPS: Array[String] = [
	"control_accel_response_delay",
	"control_brake_response_delay",
	"control_steering_response_delay",
	"control_accel_time_constant",
	"control_brake_time_constant",
	"control_steering_time_constant",
	"control_full_brake_decel",
	"control_cmd_timeout_sec",
]

const ACTUATION_CMD_PARAM_PROPS: Array[String] = [
	"actuation_accel_response_delay",
	"actuation_brake_response_delay",
	"actuation_steering_response_delay",
	"actuation_accel_time_constant",
	"actuation_brake_time_constant",
	"actuation_steering_time_constant",
	"actuation_full_brake_decel",
	"actuation_accel_full_scale",
	"actuation_cmd_timeout_sec",
]

const ACTUATION_CMD_EXTRA_PROPS: Array[String] = [
]

const SENSOR_DELAY_PARAM_PROPS: Array[String] = [
	"odom_delay",
	"velocity_delay",
	"steering_delay",
	"accel_delay",
	"tf_delay",
]

## Sensor output delays [s]. Each topic's data is buffered and published
## after the specified delay to simulate real sensor latency.
@export_group("Input Source")
@export_enum("Control Cmd", "Actuation Cmd") var preferred_input_source: int = INPUT_SOURCE_CONTROL_CMD
@export var control_cmd_timeout_sec: float = 0.5

@export_group("Control Cmd Model")
@export var control_accel_response_delay: float = 0.11
@export var control_brake_response_delay: float = 0.33
@export var control_steering_response_delay: float = 0.0
@export var control_accel_time_constant: float = 0.15
@export var control_brake_time_constant: float = 0.5
@export var control_steering_time_constant: float = 0.21
@export var control_full_brake_decel: float = 0.94

@export_group("Actuation Cmd Model")
@export var actuation_accel_response_delay: float = 0.1
@export var actuation_brake_response_delay: float = 0.1
@export var actuation_steering_response_delay: float = 0.17
@export var actuation_accel_time_constant: float = 0.1
@export var actuation_brake_time_constant: float = 0.1
@export var actuation_steering_time_constant: float = 0.15
@export var actuation_full_brake_decel: float = 0.94
@export var actuation_accel_full_scale: float = 350.0  ## accel_cmd value that approximates full drive effort
@export var actuation_cmd_timeout_sec: float = 0.5

@export_group("Sensor Delay")
@export var odom_delay: float = 0.0       ## /localization/kinematic_state
@export var velocity_delay: float = 0.0   ## /vehicle/status/velocity_status
@export var steering_delay: float = 0.0   ## /vehicle/status/steering_status
@export var accel_delay: float = 0.0      ## /localization/acceleration
@export var tf_delay: float = 0.0         ## /tf

var _ws: WebSocketPeer
var _connected: bool = false
var _timer: float = 0.0
var _advertised: Dictionary = {}
var _subscribed: Dictionary = {}

# Autoware state — default to MANUAL so keyboard works immediately
var control_mode: int = 4  # 4=MANUAL
var engaged: bool = false
var current_turn_indicator: int = 1  # DISABLE
var current_hazard_lights: int = 1   # DISABLE
var _auto_steer_cmd: float = 0.0     # rad
var _auto_accel_cmd: float = 0.0     # m/s²
var _auto_velocity_cmd: float = 0.0  # m/s
var _auto_gear_cmd: int = 2          # DRIVE
var _last_control_cmd_time_sec: float = -1.0
var _auto_actuation_accel_cmd: float = 0.0
var _auto_actuation_brake_cmd: float = 0.0
var _auto_actuation_steer_cmd: float = 0.0
var _last_actuation_cmd_time_sec: float = -1.0
var _active_response_profile: int = INPUT_SOURCE_NONE

# For acceleration computation
var _prev_vx: float = 0.0
var _prev_vy: float = 0.0

# Delay buffers: Array of [emit_time: float, json_data: Dictionary]
var _delay_queues: Dictionary = {}  # topic -> Array of [time, data]

# Autoware gear constants
const AW_GEAR_NONE = 0
const AW_GEAR_NEUTRAL = 1
const AW_GEAR_DRIVE = 2
const AW_GEAR_REVERSE = 20
const AW_GEAR_PARK = 22

# Autoware control mode constants
const AW_MODE_AUTONOMOUS = 1
const AW_MODE_MANUAL = 4

const PARAM_PROPS: Array[String] = [
	"preferred_input_source",
	"control_cmd_timeout_sec",
	"control_accel_response_delay",
	"control_brake_response_delay",
	"control_steering_response_delay",
	"control_accel_time_constant",
	"control_brake_time_constant",
	"control_steering_time_constant",
	"control_full_brake_decel",
	"actuation_accel_response_delay",
	"actuation_brake_response_delay",
	"actuation_steering_response_delay",
	"actuation_accel_time_constant",
	"actuation_brake_time_constant",
	"actuation_steering_time_constant",
	"actuation_full_brake_decel",
	"actuation_accel_full_scale",
	"actuation_cmd_timeout_sec",
	"odom_delay",
	"velocity_delay",
	"steering_delay",
	"accel_delay",
	"tf_delay",
]

func load_params_from_json(path: String) -> bool:
	if path.is_empty():
		return false
	var file = FileAccess.open(path, FileAccess.READ)
	if not file:
		print("[ROS bridge] Cannot open params file: %s" % path)
		return false
	var json = JSON.new()
	if json.parse(file.get_as_text()) != OK or not json.data is Dictionary:
		print("[ROS bridge] Failed to parse params file: %s" % path)
		return false
	var count := 0
	var root: Dictionary = json.data
	count += _load_param_section(root.get("input_source", {}), ["preferred_input_source"])
	count += _load_param_section(root.get("control_cmd", {}), CONTROL_CMD_PARAM_PROPS)
	count += _load_param_section(root.get("actuation_cmd", {}),
		ACTUATION_CMD_PARAM_PROPS + ACTUATION_CMD_EXTRA_PROPS)
	count += _load_param_section(root.get("sensor_delay", {}), SENSOR_DELAY_PARAM_PROPS)
	count += _load_legacy_params(root)
	refresh_active_response_profile()
	print("[ROS bridge] Loaded %d params from %s" % [count, path])
	return count > 0

func _ready():
	_ws = WebSocketPeer.new()
	_ws.inbound_buffer_size = 16 * 1024 * 1024  # 16 MB
	_ws.outbound_buffer_size = 16 * 1024 * 1024
	_ws.max_queued_packets = 4096
	refresh_active_response_profile()
	_connect_to_rosbridge()

func _connect_to_rosbridge():
	var err = _ws.connect_to_url(rosbridge_url)
	if err != OK:
		push_warning("ROS bridge: failed to initiate connection to %s" % rosbridge_url)

func _process(delta):
	_flush_delay_queues()
	_ws.poll()
	var state = _ws.get_ready_state()

	match state:
		WebSocketPeer.STATE_OPEN:
			if not _connected:
				_connected = true
				print("ROS bridge: connected to %s" % rosbridge_url)
				_setup_topics()
			_receive_messages()
			# Don't publish vehicle status until map is loaded (avoids flooding rosbridge)
			var map_ready = (lanelet_map == null or not is_instance_valid(lanelet_map)
				or lanelet_map._built)
			_timer += delta
			var interval = 1.0 / publish_rate_hz
			if _timer >= interval and car and is_instance_valid(car) and map_ready:
				_timer -= interval
				_publish_all()

		WebSocketPeer.STATE_CLOSING:
			pass

		WebSocketPeer.STATE_CLOSED:
			if _connected:
				_connected = false
				print("ROS bridge: disconnected (code=%d)" % _ws.get_close_code())
			_timer += delta
			if _timer > 3.0:
				_timer = 0.0
				_connect_to_rosbridge()

func _physics_process(_delta):
	if not car or not is_instance_valid(car):
		return
	# Apply autonomous control only in AUTONOMOUS mode
	if control_mode == AW_MODE_AUTONOMOUS:
		_apply_autoware_control()

func is_autonomous() -> bool:
	return control_mode == AW_MODE_AUTONOMOUS

func set_control_mode(mode: int):
	control_mode = mode
	if car and is_instance_valid(car):
		car.input_enabled = (mode == AW_MODE_MANUAL)
	var mode_name = "AUTONOMOUS" if mode == AW_MODE_AUTONOMOUS else "MANUAL"
	print("ROS bridge: control mode → %s" % mode_name)

func toggle_control_mode():
	if control_mode == AW_MODE_AUTONOMOUS:
		set_control_mode(AW_MODE_MANUAL)
	else:
		set_control_mode(AW_MODE_AUTONOMOUS)

func _apply_autoware_control():
	# Gear
	match _auto_gear_cmd:
		AW_GEAR_PARK:
			car.set_gear(car.Gear.PARK)
		AW_GEAR_REVERSE, 21:
			car.set_gear(car.Gear.REVERSE)
		AW_GEAR_NEUTRAL:
			car.set_gear(car.Gear.NEUTRAL)
		_:
			car.set_gear(car.Gear.DRIVE)

	var active_source = _resolve_input_source_mode()
	_apply_response_profile(active_source if active_source != INPUT_SOURCE_NONE
		else _profile_source_from_preference())

	match active_source:
		INPUT_SOURCE_ACTUATION_CMD:
			_apply_actuation_command()
		INPUT_SOURCE_CONTROL_CMD:
			_apply_control_command()
		_:
			car.cmd_throttle = 0.0
			car.cmd_brake = 0.0
			car.cmd_steering = 0.0

	_apply_signal_commands()
	car.input_enabled = false

func _apply_control_command():
	car.cmd_steering = clampf(_auto_steer_cmd / maxf(car.max_steer_angle, 0.001), -1.0, 1.0)
	var accel = _auto_accel_cmd
	if car.current_gear == car.Gear.REVERSE:
		accel = -accel
	_apply_longitudinal_accel_target(accel, control_full_brake_decel)

func _apply_longitudinal_accel_target(target_accel: float, full_scale_brake_decel: float):
	if target_accel >= 0.0:
		var speed = absf(car.get_forward_speed())
		var force_needed = (car.mass * target_accel + car.get_resistance_force(speed)) / 2.0
		var eff = maxf(car.drivetrain_efficiency, 0.1)
		car.cmd_throttle = clampf(force_needed / (car.max_engine_force * eff), 0.0, 1.0)
		car.cmd_brake = 0.0
		return

	car.cmd_throttle = 0.0
	var speed = absf(car.get_forward_speed())
	var coast = car.rolling_resistance_coeff * 9.81 \
		+ 0.5 * car.air_density * car.drag_coefficient * car.frontal_area * speed * speed / car.mass \
		+ car.engine_braking_force * clampf(speed / 10.0, 0.1, 1.0) / car.mass
	var brake_decel = maxf(absf(target_accel) - coast, 0.0)
	var brake_range = maxf(full_scale_brake_decel - coast, 0.01)
	car.cmd_brake = clampf(brake_decel / brake_range, 0.0, 1.0)

func _has_recent_actuation_cmd() -> bool:
	if _last_actuation_cmd_time_sec < 0.0:
		return false
	var now_sec = Time.get_ticks_msec() * 0.001
	return (now_sec - _last_actuation_cmd_time_sec) <= actuation_cmd_timeout_sec

func _has_recent_control_cmd() -> bool:
	if _last_control_cmd_time_sec < 0.0:
		return false
	var now_sec = Time.get_ticks_msec() * 0.001
	return (now_sec - _last_control_cmd_time_sec) <= control_cmd_timeout_sec

func _apply_actuation_command():
	car.cmd_steering = clampf(
		_auto_actuation_steer_cmd / maxf(car.max_steer_angle, 0.001), -1.0, 1.0)
	var accel = _get_actuation_target_accel()
	if car.current_gear == car.Gear.REVERSE:
		accel = -accel
	_apply_longitudinal_accel_target(accel, actuation_full_brake_decel)

func _get_actuation_target_accel() -> float:
	if _auto_actuation_brake_cmd > 0.0:
		return -_auto_actuation_brake_cmd

	var normalized = clampf(_auto_actuation_accel_cmd / maxf(actuation_accel_full_scale, 1.0), 0.0, 1.0)
	return normalized * maxf(car.max_engine_force * car.drivetrain_efficiency * 2.0 / car.mass, 0.0)

func _resolve_input_source_mode() -> int:
	match preferred_input_source:
		INPUT_SOURCE_CONTROL_CMD:
			return INPUT_SOURCE_CONTROL_CMD if _has_recent_control_cmd() else INPUT_SOURCE_NONE
		_:
			return INPUT_SOURCE_ACTUATION_CMD if _has_recent_actuation_cmd() else INPUT_SOURCE_NONE
	return INPUT_SOURCE_NONE

func _profile_source_from_preference() -> int:
	return INPUT_SOURCE_CONTROL_CMD if preferred_input_source == INPUT_SOURCE_CONTROL_CMD else INPUT_SOURCE_ACTUATION_CMD

func refresh_active_response_profile():
	_active_response_profile = INPUT_SOURCE_NONE
	_apply_response_profile(_profile_source_from_preference())

func _apply_response_profile(source: int):
	if not car or not is_instance_valid(car):
		return
	if source == _active_response_profile:
		return
	match source:
		INPUT_SOURCE_ACTUATION_CMD:
			car.accel_response_delay = actuation_accel_response_delay
			car.brake_response_delay = actuation_brake_response_delay
			car.steering_response_delay = actuation_steering_response_delay
			car.accel_time_constant = actuation_accel_time_constant
			car.brake_time_constant = actuation_brake_time_constant
			car.steering_time_constant = actuation_steering_time_constant
		_:
			car.accel_response_delay = control_accel_response_delay
			car.brake_response_delay = control_brake_response_delay
			car.steering_response_delay = control_steering_response_delay
			car.accel_time_constant = control_accel_time_constant
			car.brake_time_constant = control_brake_time_constant
			car.steering_time_constant = control_steering_time_constant
	car.apply_vehicle_params()
	_active_response_profile = source

func set_preferred_input_source(mode: int):
	preferred_input_source = INPUT_SOURCE_CONTROL_CMD if mode == INPUT_SOURCE_CONTROL_CMD else INPUT_SOURCE_ACTUATION_CMD
	refresh_active_response_profile()

func get_preferred_input_source_key() -> String:
	return _input_source_key(preferred_input_source)

func get_preferred_input_source_label() -> String:
	return _input_source_label(preferred_input_source)

func get_active_input_source() -> int:
	return _resolve_input_source_mode()

func get_active_input_source_label() -> String:
	return _input_source_label(_resolve_input_source_mode())

func get_input_source_status() -> String:
	match _resolve_input_source_mode():
		INPUT_SOURCE_CONTROL_CMD:
			return "control_cmd active"
		INPUT_SOURCE_ACTUATION_CMD:
			return "actuation_cmd active"
		_:
			return "waiting control_cmd" if preferred_input_source == INPUT_SOURCE_CONTROL_CMD else "waiting actuation_cmd"

func _input_source_key(mode: int) -> String:
	match mode:
		INPUT_SOURCE_CONTROL_CMD:
			return "control_cmd"
		INPUT_SOURCE_ACTUATION_CMD:
			return "actuation_cmd"
		_:
			return "actuation_cmd"

func _input_source_label(mode: int) -> String:
	match mode:
		INPUT_SOURCE_CONTROL_CMD:
			return "CONTROL"
		INPUT_SOURCE_ACTUATION_CMD:
			return "ACTUATION"
		INPUT_SOURCE_NONE:
			return "NONE"
		_:
			return "ACTUATION"

func _load_param_section(section, props: Array[String]) -> int:
	if not (section is Dictionary):
		return 0
	var count := 0
	for prop in props:
		if not section.has(prop):
			continue
		if prop == "preferred_input_source":
			set_preferred_input_source(_parse_input_source(section[prop]))
		else:
			_assign_loaded_value(prop, section[prop])
		count += 1
	return count

func _load_legacy_params(root: Dictionary) -> int:
	var count := 0
	var control_section = root.get("control_cmd", {})
	var actuation_section = root.get("actuation_cmd", {})
	var input_section = root.get("input_source", {})
	var sensor_section = root.get("sensor_delay", {})
	count += _load_legacy_value(root, control_section, "accel_response_delay", "control_accel_response_delay")
	count += _load_legacy_value(root, control_section, "brake_response_delay", "control_brake_response_delay")
	count += _load_legacy_value(root, control_section, "steering_response_delay", "control_steering_response_delay")
	count += _load_legacy_value(root, control_section, "accel_time_constant", "control_accel_time_constant")
	count += _load_legacy_value(root, control_section, "brake_time_constant", "control_brake_time_constant")
	count += _load_legacy_value(root, control_section, "steering_time_constant", "control_steering_time_constant")
	count += _load_legacy_value(root, control_section, "full_brake_decel", "control_full_brake_decel")
	count += _load_legacy_value(root, control_section, "control_cmd_timeout_sec", "control_cmd_timeout_sec")
	count += _load_legacy_value(root, actuation_section, "accel_response_delay", "actuation_accel_response_delay")
	count += _load_legacy_value(root, actuation_section, "brake_response_delay", "actuation_brake_response_delay")
	count += _load_legacy_value(root, actuation_section, "steering_response_delay", "actuation_steering_response_delay")
	count += _load_legacy_value(root, actuation_section, "accel_time_constant", "actuation_accel_time_constant")
	count += _load_legacy_value(root, actuation_section, "brake_time_constant", "actuation_brake_time_constant")
	count += _load_legacy_value(root, actuation_section, "steering_time_constant", "actuation_steering_time_constant")
	count += _load_legacy_value(root, actuation_section, "full_brake_decel", "actuation_full_brake_decel")
	count += _load_legacy_value(root, actuation_section, "actuation_accel_full_scale", "actuation_accel_full_scale")
	count += _load_legacy_value(root, actuation_section, "actuation_cmd_timeout_sec", "actuation_cmd_timeout_sec")
	count += _load_legacy_value(root, input_section, "preferred_input_source", "preferred_input_source")
	for prop in SENSOR_DELAY_PARAM_PROPS:
		count += _load_legacy_value(root, sensor_section, prop, prop)
	return count

func _load_legacy_value(root: Dictionary, section, legacy_key: String, prop: String) -> int:
	if not root.has(legacy_key):
		return 0
	if (section is Dictionary) and section.has(prop):
		return 0
	if prop == "preferred_input_source":
		set_preferred_input_source(_parse_input_source(root[legacy_key]))
	else:
		_assign_loaded_value(prop, root[legacy_key])
	return 1

func _assign_loaded_value(prop: String, value):
	var current = get(prop)
	match typeof(current):
		TYPE_INT:
			set(prop, int(value))
		TYPE_BOOL:
			set(prop, bool(value))
		TYPE_STRING:
			set(prop, str(value))
		_:
			set(prop, float(value))

func _parse_input_source(value) -> int:
	if typeof(value) == TYPE_STRING:
		match str(value).to_lower():
			"control", "control_cmd":
				return INPUT_SOURCE_CONTROL_CMD
			"actuation", "actuation_cmd":
				return INPUT_SOURCE_ACTUATION_CMD
			_:
				return INPUT_SOURCE_ACTUATION_CMD
	return INPUT_SOURCE_CONTROL_CMD if int(value) == INPUT_SOURCE_CONTROL_CMD else INPUT_SOURCE_ACTUATION_CMD

func _apply_signal_commands():
	# Turn indicators: 1=DISABLE, 2=LEFT, 3=RIGHT
	match current_turn_indicator:
		2:
			car.current_turn_signal = car.TurnSignal.LEFT
		3:
			car.current_turn_signal = car.TurnSignal.RIGHT
		_:
			car.current_turn_signal = car.TurnSignal.OFF

	# Hazard lights: 1=DISABLE, 2=ENABLE
	car.hazard_lights = (current_hazard_lights == 2)
	if car.hazard_lights:
		car.current_turn_signal = car.TurnSignal.OFF

# ============================================================
# Topic setup
# ============================================================

func _setup_topics():
	# Advertise output topics
	_advertise("/tf", "tf2_msgs/msg/TFMessage")
	_advertise("/localization/kinematic_state", "nav_msgs/msg/Odometry")
	_advertise("/vehicle/status/velocity_status", "autoware_vehicle_msgs/msg/VelocityReport")
	_advertise("/vehicle/status/steering_status", "autoware_vehicle_msgs/msg/SteeringReport")
	_advertise("/vehicle/status/gear_status", "autoware_vehicle_msgs/msg/GearReport")
	_advertise("/vehicle/status/control_mode", "autoware_vehicle_msgs/msg/ControlModeReport")
	_advertise("/vehicle/status/turn_indicators_status", "autoware_vehicle_msgs/msg/TurnIndicatorsReport")
	_advertise("/vehicle/status/hazard_lights_status", "autoware_vehicle_msgs/msg/HazardLightsReport")
	_advertise("/localization/acceleration", "geometry_msgs/msg/AccelWithCovarianceStamped")
	# Keep marker for RViz debugging
	_advertise("/vehicle/marker", "visualization_msgs/msg/Marker")

	# Subscribe to input topics
	_subscribe("/control/command/control_cmd", "autoware_control_msgs/msg/Control")
	_subscribe("/control/command/actuation_cmd",
		"tier4_vehicle_msgs/msg/ActuationCommandStamped")
	_subscribe("/control/command/gear_cmd", "autoware_vehicle_msgs/msg/GearCommand")
	_subscribe("/control/command/turn_indicators_cmd", "autoware_vehicle_msgs/msg/TurnIndicatorsCommand")
	_subscribe("/control/command/hazard_lights_cmd", "autoware_vehicle_msgs/msg/HazardLightsCommand")
	_subscribe("/initialpose3d", "geometry_msgs/msg/PoseWithCovarianceStamped")
	_subscribe("/vehicle/engage", "autoware_vehicle_msgs/msg/Engage")
	_subscribe("/tf", "tf2_msgs/msg/TFMessage")
	_subscribe("/tf_static", "tf2_msgs/msg/TFMessage")
	_subscribe("/perception/traffic_light_recognition/traffic_signals",
		"autoware_perception_msgs/msg/TrafficLightGroupArray")
	_subscribe("/planning/trajectory",
		"autoware_planning_msgs/msg/Trajectory")
	_subscribe("/planning/scenario_planning/trajectory",
		"autoware_planning_msgs/msg/Trajectory")
	_subscribe("/perception/object_recognition/objects",
		"autoware_perception_msgs/msg/PredictedObjects")

	# Advertise control_mode_request service
	_send_json({
		"op": "advertise_service",
		"service": "/control/control_mode_request",
		"type": "autoware_vehicle_msgs/srv/ControlModeCommand"
	})
	print("ROS bridge: advertised service /control/control_mode_request")

func _advertise(topic: String, msg_type: String):
	_send_json({"op": "advertise", "topic": topic, "type": msg_type})
	_advertised[topic] = true
	print("ROS bridge: advertised %s" % topic)

func _subscribe(topic: String, msg_type: String):
	_send_json({"op": "subscribe", "topic": topic, "type": msg_type})
	_subscribed[topic] = true
	print("ROS bridge: subscribed %s" % topic)

# ============================================================
# Receive and dispatch
# ============================================================

func _receive_messages():
	while _ws.get_available_packet_count() > 0:
		var pkt = _ws.get_packet()
		var text = pkt.get_string_from_utf8()
		var json = JSON.new()
		if json.parse(text) != OK:
			continue
		var data = json.data
		if not data is Dictionary:
			continue

		var op = data.get("op", "")
		match op:
			"publish":
				_handle_topic_msg(data.get("topic", ""), data.get("msg", {}))
			"call_service":
				_handle_service_call(data)
			"service_response":
				_handle_service_response(data)

func _handle_topic_msg(topic: String, msg: Dictionary):
	match topic:
		"/control/command/control_cmd":
			var lat = msg.get("lateral", {})
			var lon = msg.get("longitudinal", {})
			_auto_steer_cmd = lat.get("steering_tire_angle", 0.0)
			_auto_accel_cmd = lon.get("acceleration", 0.0)
			_auto_velocity_cmd = lon.get("velocity", 0.0)
			_last_control_cmd_time_sec = Time.get_ticks_msec() * 0.001

		"/control/command/actuation_cmd":
			var act = msg.get("actuation", {})
			_auto_actuation_accel_cmd = float(act.get("accel_cmd", 0.0))
			_auto_actuation_brake_cmd = float(act.get("brake_cmd", 0.0))
			_auto_actuation_steer_cmd = float(act.get("steer_cmd", 0.0))
			_last_actuation_cmd_time_sec = Time.get_ticks_msec() * 0.001

		"/control/command/gear_cmd":
			_auto_gear_cmd = int(msg.get("command", AW_GEAR_DRIVE))

		"/control/command/turn_indicators_cmd":
			current_turn_indicator = int(msg.get("command", 1))

		"/control/command/hazard_lights_cmd":
			current_hazard_lights = int(msg.get("command", 1))

		"/vehicle/engage":
			var eng = msg.get("engage", false)
			engaged = eng
			if eng:
				set_control_mode(AW_MODE_AUTONOMOUS)
			else:
				set_control_mode(AW_MODE_MANUAL)

		"/initialpose3d":
			_handle_initial_pose(msg)

		"/tf", "/tf_static":
			_handle_incoming_tf(msg)

		"/perception/traffic_light_recognition/traffic_signals":
			_handle_traffic_light_recognition(msg)

		"/planning/trajectory", "/planning/scenario_planning/trajectory":
			_handle_trajectory(msg)

		"/perception/object_recognition/objects":
			_handle_dynamic_objects(msg)

func _handle_incoming_tf(msg: Dictionary):
	var transforms = msg.get("transforms", [])
	for tf in transforms:
		var parent = tf.get("header", {}).get("frame_id", "")
		var child = tf.get("child_frame_id", "")
		if parent == "map" and child == "viewer":
			var t = tf.get("transform", {}).get("translation", {})
			viewer_offset_x = t.get("x", 0.0)
			viewer_offset_y = t.get("y", 0.0)
			viewer_offset_z = t.get("z", 0.0)
			if not viewer_offset_valid:
				viewer_offset_valid = true
				print("ROS bridge: viewer frame offset = (%.1f, %.1f, %.1f)" % [
					viewer_offset_x, viewer_offset_y, viewer_offset_z])

## Convert ROS map-frame coords to Godot viewer-frame coords
func ros_map_to_godot(ros_x: float, ros_y: float, ros_z: float) -> Vector3:
	var vx = ros_x - viewer_offset_x
	var vy = ros_y - viewer_offset_y
	var vz = ros_z - viewer_offset_z
	return Vector3(-vy, vz, -vx)

## Convert Godot viewer-frame coords to ROS map-frame coords
func godot_to_ros_map(godot_pos: Vector3) -> Dictionary:
	var ros_x = -godot_pos.z + viewer_offset_x
	var ros_y = -godot_pos.x + viewer_offset_y
	var ros_z = godot_pos.y + viewer_offset_z
	return {"x": ros_x, "y": ros_y, "z": ros_z}

func _handle_dynamic_objects(msg: Dictionary):
	if not dynamic_object_mesh or not is_instance_valid(dynamic_object_mesh):
		return
	if not viewer_offset_valid:
		return
	var objects_raw = msg.get("objects", [])
	if typeof(objects_raw) != TYPE_ARRAY:
		return
	var objects: Array = []
	for obj in objects_raw:
		if typeof(obj) != TYPE_DICTIONARY:
			continue
		var kin = obj.get("kinematics", {})
		var pose = kin.get("initial_pose_with_covariance", {}).get("pose", {})
		var p = pose.get("position", {})
		var q = pose.get("orientation", {})
		var godot_pos = ros_map_to_godot(
			float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("z", 0)))
		var rqx = float(q.get("x", 0))
		var rqy = float(q.get("y", 0))
		var rqz = float(q.get("z", 0))
		var rqw = float(q.get("w", 1))
		var godot_quat = Quaternion(-rqy, rqz, -rqx, rqw)

		var shape = obj.get("shape", {})
		var shape_type = int(shape.get("type", 0))
		var dims_raw = shape.get("dimensions", {})
		var dims = Vector3(
			float(dims_raw.get("x", 1)), float(dims_raw.get("y", 1)), float(dims_raw.get("z", 1)))

		var footprint: Array = []
		if shape_type == 2:  # POLYGON
			var fp = shape.get("footprint", {}).get("points", [])
			if typeof(fp) == TYPE_ARRAY:
				for pt in fp:
					footprint.append(Vector2(float(pt.get("x", 0)), float(pt.get("y", 0))))

		objects.append({
			"position": godot_pos,
			"quaternion": godot_quat,
			"shape_type": shape_type,
			"dimensions": dims,
			"footprint": footprint,
		})
	dynamic_object_mesh.set_objects(objects)

func _handle_trajectory(msg: Dictionary):
	if not trajectory_mesh or not is_instance_valid(trajectory_mesh):
		return
	if not viewer_offset_valid:
		return
	var pts_raw = msg.get("points", [])
	if typeof(pts_raw) != TYPE_ARRAY or pts_raw.is_empty():
		return
	var points: Array = []
	for pt in pts_raw:
		if typeof(pt) != TYPE_DICTIONARY:
			continue
		var pose = pt.get("pose", {})
		var p = pose.get("position", {})
		var q = pose.get("orientation", {})
		var godot_pos = ros_map_to_godot(
			float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("z", 0)))
		# ROS quat (x,y,z,w) → Godot quat
		var rqx = float(q.get("x", 0))
		var rqy = float(q.get("y", 0))
		var rqz = float(q.get("z", 0))
		var rqw = float(q.get("w", 1))
		var godot_quat = Quaternion(-rqy, rqz, -rqx, rqw)
		points.append({
			"pos": godot_pos,
			"quat": godot_quat,
			"vel": float(pt.get("longitudinal_velocity_mps", 0)),
		})
	trajectory_mesh.set_trajectory(points)

func _handle_traffic_light_recognition(msg: Dictionary):
	if not traffic_light_manager or not is_instance_valid(traffic_light_manager):
		return
	# autoware_perception_msgs/msg/TrafficLightGroupArray
	# .traffic_light_groups[] = { traffic_light_group_id, elements[] }
	var groups = msg.get("traffic_light_groups", [])
	if typeof(groups) != TYPE_ARRAY or groups.is_empty():
		return
	var status_list: Array = []
	for g in groups:
		if typeof(g) != TYPE_DICTIONARY:
			continue
		var gid = int(g.get("traffic_light_group_id", -1))
		var elements_raw = g.get("elements", [])
		if typeof(elements_raw) != TYPE_ARRAY:
			continue
		var elements: Array = []
		for e in elements_raw:
			if typeof(e) != TYPE_DICTIONARY:
				continue
			# Map Autoware enum values to string names
			var color_val = int(e.get("color", 0))
			var shape_val = int(e.get("shape", 0))
			var color_str = _traffic_color_to_string(color_val)
			var arrow_str = _traffic_shape_to_arrow(shape_val)
			elements.append({"color": color_str, "arrow": arrow_str})
		status_list.append({"group_id": gid, "status_elements": elements})
	traffic_light_manager.update_recognition(status_list)

func _traffic_color_to_string(val: int) -> String:
	# autoware_perception_msgs/msg/TrafficLightElement color constants
	match val:
		1: return "red"
		2: return "yellow"      # AMBER
		3: return "green"
		4: return "white"
		_: return "none"        # UNKNOWN=0

func _traffic_shape_to_arrow(val: int) -> String:
	# autoware_perception_msgs/msg/TrafficLightElement shape constants
	match val:
		1: return "none"        # CIRCLE
		2: return "left"        # LEFT_ARROW
		3: return "right"       # RIGHT_ARROW
		4: return "up"          # UP_ARROW
		5: return "down"        # DOWN_ARROW
		6: return "none"        # CROSS
		_: return "none"        # UNKNOWN=0

func _handle_service_response(data: Dictionary):
	var id = data.get("id", "")
	var values = data.get("values", {})
	if not values is Dictionary:
		values = {}
	# Forward to lanelet_map if it's a lanelet-related response
	if lanelet_map and is_instance_valid(lanelet_map):
		if id in ["lanelet_batch_count", "lanelet_batch"]:
			lanelet_map.handle_service_response(id, values)


func _handle_initial_pose(msg: Dictionary):
	if not car or not is_instance_valid(car):
		return
	var pose = msg.get("pose", {}).get("pose", {})
	var p = pose.get("position", {})
	var q = pose.get("orientation", {})

	# Convert from ROS map frame to Godot viewer frame
	var godot_pos = ros_map_to_godot(p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0))
	godot_pos.y += 1.0  # lift slightly

	car.global_position = godot_pos
	car.linear_velocity = Vector3.ZERO
	car.angular_velocity = Vector3.ZERO

	var rqx = q.get("x", 0.0)
	var rqy = q.get("y", 0.0)
	var rqz = q.get("z", 0.0)
	var rqw = q.get("w", 1.0)
	var godot_quat = Quaternion(-rqy, rqz, -rqx, rqw)
	car.global_transform.basis = Basis(godot_quat)

	print("ROS bridge: initial pose → godot (%.1f, %.1f, %.1f)" % [godot_pos.x, godot_pos.y, godot_pos.z])

func _handle_service_call(data: Dictionary):
	var service = data.get("service", "")
	var id = data.get("id", "")

	if service == "/control/control_mode_request":
		var args = data.get("args", {})
		var mode = int(args.get("mode", 0))
		set_control_mode(mode)
		_send_json({
			"op": "service_response",
			"service": service,
			"id": id,
			"values": {"success": true},
			"result": true
		})

# ============================================================
# Publish all output topics
# ============================================================

func _publish_all():
	var now = _ros_time_now()
	var bl = _get_base_link_transform()
	_publish_tf(now, bl)
	_publish_odom(now, bl)
	_publish_velocity_status(now)
	_publish_steering_status(now)
	_publish_gear_status(now)
	_publish_control_mode_report(now)
	_publish_turn_indicators_report(now)
	_publish_hazard_lights_report(now)
	_publish_acceleration(now)
	_publish_marker(now, bl)

func _get_base_link_transform() -> Dictionary:
	var rl = car.get_node("WheelRL").global_position
	var rr = car.get_node("WheelRR").global_position
	var rear_axle = (rl + rr) / 2.0
	var contact = rear_axle - Vector3(0, car.wheel_radius_param, 0)

	var ros_pos = godot_to_ros_map(contact)
	var quat = car.global_transform.basis.get_rotation_quaternion()
	return {
		"pos": ros_pos,
		"quat": {"x": -quat.z, "y": -quat.x, "z": quat.y, "w": quat.w}
	}

func _publish_tf(now: Dictionary, bl: Dictionary):
	_send_delayed({
		"op": "publish",
		"topic": "/tf",
		"msg": {
			"transforms": [{
				"header": {"stamp": now, "frame_id": "map"},
				"child_frame_id": "base_link",
				"transform": {
					"translation": bl["pos"],
					"rotation": bl["quat"]
				}
			}]
		}
	}, tf_delay)

func _publish_odom(now: Dictionary, bl: Dictionary):
	var vel = car.linear_velocity
	var forward = -car.global_transform.basis.z
	var right = car.global_transform.basis.x
	var vx = vel.dot(forward)
	var vy = -vel.dot(right)
	var yaw_rate = car.angular_velocity.y

	_send_delayed({
		"op": "publish",
		"topic": "/localization/kinematic_state",
		"msg": {
			"header": {"stamp": now, "frame_id": "map"},
			"child_frame_id": "base_link",
			"pose": {
				"pose": {
					"position": bl["pos"],
					"orientation": bl["quat"]
				},
				"covariance": _zero_cov()
			},
			"twist": {
				"twist": {
					"linear": {"x": vx, "y": vy, "z": 0.0},
					"angular": {"x": 0.0, "y": 0.0, "z": yaw_rate}
				},
				"covariance": _zero_cov()
			}
		}
	}, odom_delay)

func _publish_velocity_status(now: Dictionary):
	var vel = car.linear_velocity
	var forward = -car.global_transform.basis.z
	var right = car.global_transform.basis.x
	_send_delayed({
		"op": "publish",
		"topic": "/vehicle/status/velocity_status",
		"msg": {
			"header": {"stamp": now, "frame_id": "base_link"},
			"longitudinal_velocity": vel.dot(forward),
			"lateral_velocity": vel.dot(right),
			"heading_rate": car.angular_velocity.y
		}
	}, velocity_delay)

func _publish_steering_status(now: Dictionary):
	_send_delayed({
		"op": "publish",
		"topic": "/vehicle/status/steering_status",
		"msg": {
			"stamp": now,
			"steering_tire_angle": car.steering
		}
	}, steering_delay)

func _publish_gear_status(now: Dictionary):
	var aw_gear = AW_GEAR_DRIVE
	match car.current_gear:
		car.Gear.PARK: aw_gear = AW_GEAR_PARK
		car.Gear.REVERSE: aw_gear = AW_GEAR_REVERSE
		car.Gear.NEUTRAL: aw_gear = AW_GEAR_NEUTRAL
		car.Gear.DRIVE: aw_gear = AW_GEAR_DRIVE
	_send_json({
		"op": "publish",
		"topic": "/vehicle/status/gear_status",
		"msg": {"stamp": now, "report": aw_gear}
	})

func _publish_control_mode_report(now: Dictionary):
	_send_json({
		"op": "publish",
		"topic": "/vehicle/status/control_mode",
		"msg": {"stamp": now, "mode": control_mode}
	})

func _publish_turn_indicators_report(now: Dictionary):
	# Sync from car state (manual) or Autoware command (auto)
	var report = current_turn_indicator
	if car and is_instance_valid(car) and car.input_enabled:
		match car.current_turn_signal:
			car.TurnSignal.OFF: report = 1   # DISABLE
			car.TurnSignal.LEFT: report = 2  # ENABLE_LEFT
			car.TurnSignal.RIGHT: report = 3 # ENABLE_RIGHT
	_send_json({
		"op": "publish",
		"topic": "/vehicle/status/turn_indicators_status",
		"msg": {"stamp": now, "report": report}
	})

func _publish_hazard_lights_report(now: Dictionary):
	var report = current_hazard_lights
	if car and is_instance_valid(car) and car.input_enabled:
		report = 2 if car.hazard_lights else 1  # ENABLE / DISABLE
	_send_json({
		"op": "publish",
		"topic": "/vehicle/status/hazard_lights_status",
		"msg": {"stamp": now, "report": report}
	})

func _publish_acceleration(now: Dictionary):
	var vel = car.linear_velocity
	var forward = -car.global_transform.basis.z
	var right = car.global_transform.basis.x
	var vx = vel.dot(forward)
	var vy = vel.dot(right)
	var wz = car.angular_velocity.y
	var dt = 1.0 / publish_rate_hz

	# ax = d(vx)/dt, ay = wz * vx (same as Autoware simple_planning_simulator)
	var ax = (vx - _prev_vx) / dt if dt > 0.001 else 0.0
	var ay = wz * vx
	_prev_vx = vx
	_prev_vy = vy

	var cov = _zero_cov()
	cov[0] = 0.001   # X_X
	cov[7] = 0.001   # Y_Y
	cov[14] = 0.001  # Z_Z
	cov[21] = 0.001  # ROLL_ROLL
	cov[28] = 0.001  # PITCH_PITCH
	cov[35] = 0.001  # YAW_YAW

	_send_delayed({
		"op": "publish",
		"topic": "/localization/acceleration",
		"msg": {
			"header": {"stamp": now, "frame_id": "base_link"},
			"accel": {
				"accel": {
					"linear": {"x": ax, "y": ay, "z": 0.0},
					"angular": {"x": 0.0, "y": 0.0, "z": 0.0}
				},
				"covariance": cov
			}
		}
	}, accel_delay)

func _publish_marker(now: Dictionary, bl: Dictionary):
	_send_json({
		"op": "publish",
		"topic": "/vehicle/marker",
		"msg": {
			"header": {"stamp": now, "frame_id": "map"},
			"ns": "vehicle", "id": 0, "type": 1, "action": 0,
			"pose": {
				"position": {"x": bl["pos"]["x"], "y": bl["pos"]["y"], "z": bl["pos"]["z"] + 1.0},
				"orientation": bl["quat"]
			},
			"scale": {
				"x": float(car.front_overhang + car.wheel_base + car.rear_overhang),
				"y": float(car.tread + 0.36),
				"z": 2.1
			},
			"color": {"r": 0.15, "g": 0.55, "b": 0.25, "a": 0.9},
			"lifetime": {"sec": 0, "nanosec": int(2.0 / publish_rate_hz * 1e9)}
		}
	})

# ============================================================
# Utilities
# ============================================================

func _ros_time_now() -> Dictionary:
	var t = Time.get_unix_time_from_system()
	var sec = int(t)
	var nsec = int((t - sec) * 1e9)
	return {"sec": sec, "nanosec": nsec}

func _zero_cov() -> Array:
	var c = []
	c.resize(36)
	c.fill(0.0)
	return c

func _send_json(data: Dictionary):
	if _connected:
		_ws.send_text(JSON.stringify(data))

func _send_delayed(data: Dictionary, delay_sec: float):
	## Queue a message to be sent after delay_sec. If delay is 0, send immediately.
	if delay_sec <= 0.001:
		_send_json(data)
		return
	var topic = data.get("topic", "")
	if not _delay_queues.has(topic):
		_delay_queues[topic] = []
	var emit_time = Time.get_unix_time_from_system() + delay_sec
	_delay_queues[topic].append([emit_time, data])

func _flush_delay_queues():
	if _delay_queues.is_empty() or not _connected:
		return
	var now = Time.get_unix_time_from_system()
	for topic in _delay_queues:
		var queue: Array = _delay_queues[topic]
		while queue.size() > 0 and queue[0][0] <= now:
			_send_json(queue[0][1])
			queue.pop_front()

func _exit_tree():
	if _connected:
		for topic in _advertised:
			_send_json({"op": "unadvertise", "topic": topic})
		for topic in _subscribed:
			_send_json({"op": "unsubscribe", "topic": topic})
		_ws.close()
