extends PanelContainer
## In-game parameter tuning panel. Toggle with Tab key.
## Handles both vehicle params (on car) and sensor delay params (on ros_bridge).

var car: VehicleBody3D
var ros_bridge: Node  # ros_bridge.gd instance
var _sliders: Dictionary = {}

signal car_rebuild_requested

const GEOM_PROPS = ["wheel_base", "tread", "front_overhang", "rear_overhang", "wheel_radius_param"]

# "target" field: "car" or "bridge" — determines which node the property lives on.
const PARAMS = [
	{"group": "Geometry (rebuilds car)"},
	{"name": "Wheel Base [m]", "prop": "wheel_base", "min": 1.5, "max": 6.0, "step": 0.05, "target": "car"},
	{"name": "Tread [m]", "prop": "tread", "min": 1.0, "max": 2.5, "step": 0.01, "target": "car"},
	{"name": "Front Overhang [m]", "prop": "front_overhang", "min": 0.3, "max": 2.0, "step": 0.05, "target": "car"},
	{"name": "Rear Overhang [m]", "prop": "rear_overhang", "min": 0.3, "max": 2.0, "step": 0.05, "target": "car"},
	{"name": "Wheel Radius [m]", "prop": "wheel_radius_param", "min": 0.2, "max": 0.6, "step": 0.01, "target": "car"},
	{"group": "Powertrain"},
	{"name": "Max Engine Force [N]", "prop": "max_engine_force", "min": 1000, "max": 20000, "step": 100, "target": "car"},
	{"name": "Max Brake Force", "prop": "max_brake_force", "min": 10, "max": 1000, "step": 10, "target": "car"},
	{"name": "Reverse Power Ratio", "prop": "reverse_power_ratio", "min": 0.1, "max": 1.0, "step": 0.05, "target": "car"},
	{"group": "Steering"},
	{"name": "Max Steer Angle [rad]", "prop": "max_steer_angle", "min": 0.1, "max": 1.0, "step": 0.01, "target": "car"},
	{"name": "Steer Speed Threshold [km/h]", "prop": "steer_speed_threshold", "min": 10, "max": 120, "step": 5, "target": "car"},
	{"name": "Steer High Speed Ratio", "prop": "steer_high_speed_ratio", "min": 0.05, "max": 1.0, "step": 0.05, "target": "car"},
	{"group": "Transport Delay [s]"},
	{"name": "Accel Delay", "prop": "accel_response_delay", "min": 0.0, "max": 1.0, "step": 0.01, "target": "car"},
	{"name": "Brake Delay", "prop": "brake_response_delay", "min": 0.0, "max": 0.5, "step": 0.01, "target": "car"},
	{"name": "Steering Delay", "prop": "steering_response_delay", "min": 0.0, "max": 0.5, "step": 0.01, "target": "car"},
	{"group": "Time Constant (1st order lag) [s]"},
	{"name": "Accel TC", "prop": "accel_time_constant", "min": 0.0, "max": 1.0, "step": 0.01, "target": "car"},
	{"name": "Brake TC", "prop": "brake_time_constant", "min": 0.0, "max": 0.5, "step": 0.01, "target": "car"},
	{"name": "Steering TC", "prop": "steering_time_constant", "min": 0.0, "max": 0.5, "step": 0.01, "target": "car"},
	{"group": "Sensor Output Delay [s]"},
	{"name": "TF Delay", "prop": "tf_delay", "min": 0.0, "max": 1.0, "step": 0.01, "target": "bridge"},
	{"name": "Odometry Delay", "prop": "odom_delay", "min": 0.0, "max": 1.0, "step": 0.01, "target": "bridge"},
	{"name": "Velocity Delay", "prop": "velocity_delay", "min": 0.0, "max": 1.0, "step": 0.01, "target": "bridge"},
	{"name": "Steering Sensor Delay", "prop": "steering_delay", "min": 0.0, "max": 1.0, "step": 0.01, "target": "bridge"},
	{"name": "Acceleration Delay", "prop": "accel_delay", "min": 0.0, "max": 1.0, "step": 0.01, "target": "bridge"},
	{"group": "Resistance"},
	{"name": "Rolling Resistance", "prop": "rolling_resistance_coeff", "min": 0.0, "max": 0.05, "step": 0.001, "target": "car"},
	{"name": "Drag Coefficient", "prop": "drag_coefficient", "min": 0.0, "max": 1.0, "step": 0.01, "target": "car"},
	{"name": "Frontal Area [m2]", "prop": "frontal_area", "min": 1.0, "max": 8.0, "step": 0.1, "target": "car"},
	{"name": "Engine Braking [N]", "prop": "engine_braking_force", "min": 0, "max": 2000, "step": 10, "target": "car"},
	{"group": "Vehicle"},
	{"name": "Vehicle Weight [kg]", "prop": "vehicle_weight", "min": 500, "max": 15000, "step": 50, "target": "car"},
	{"name": "Center of Mass Y [m]", "prop": "center_of_mass_y", "min": -1.0, "max": 2.0, "step": 0.05, "target": "car"},
	{"group": "Suspension"},
	{"name": "Rest Length [m]", "prop": "suspension_rest_length_val", "min": 0.05, "max": 0.5, "step": 0.01, "target": "car"},
	{"name": "Stiffness", "prop": "suspension_stiffness_val", "min": 10, "max": 200, "step": 1, "target": "car"},
	{"name": "Travel [m]", "prop": "suspension_travel_val", "min": 0.05, "max": 0.5, "step": 0.01, "target": "car"},
	{"name": "Max Susp Force [N]", "prop": "suspension_max_force_val", "min": 10000, "max": 200000, "step": 5000, "target": "car"},
	{"name": "Damping Compression", "prop": "damping_compression_val", "min": 0.5, "max": 10.0, "step": 0.1, "target": "car"},
	{"name": "Damping Relaxation", "prop": "damping_relaxation_val", "min": 0.5, "max": 10.0, "step": 0.1, "target": "car"},
	{"name": "Wheel Friction Slip", "prop": "wheel_friction_slip", "min": 0.5, "max": 10.0, "step": 0.1, "target": "car"},
]

func _ready():
	visible = false
	_build_ui()

func _build_ui():
	var style = StyleBoxFlat.new()
	style.bg_color = Color(0.1, 0.1, 0.12, 0.9)
	style.corner_radius_top_left = 8
	style.corner_radius_top_right = 8
	style.corner_radius_bottom_left = 8
	style.corner_radius_bottom_right = 8
	style.content_margin_left = 12
	style.content_margin_right = 12
	style.content_margin_top = 8
	style.content_margin_bottom = 8
	add_theme_stylebox_override("panel", style)

	var scroll = ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(440, 600)
	add_child(scroll)

	var vbox = VBoxContainer.new()
	vbox.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(vbox)

	var title = Label.new()
	title.text = "Vehicle Tuning [Tab to close]"
	title.add_theme_font_size_override("font_size", 18)
	title.add_theme_color_override("font_color", Color(1, 0.9, 0.3))
	vbox.add_child(title)
	vbox.add_child(HSeparator.new())

	for param in PARAMS:
		if param.has("group"):
			var lbl = Label.new()
			lbl.text = param["group"]
			lbl.add_theme_font_size_override("font_size", 15)
			lbl.add_theme_color_override("font_color", Color(0.5, 0.8, 1.0))
			vbox.add_child(lbl)
			continue

		var row = HBoxContainer.new()
		row.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		vbox.add_child(row)

		var label = Label.new()
		label.text = param["name"]
		label.custom_minimum_size.x = 185
		label.add_theme_font_size_override("font_size", 13)
		row.add_child(label)

		var slider = HSlider.new()
		slider.min_value = param["min"]
		slider.max_value = param["max"]
		slider.step = param["step"]
		slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		slider.custom_minimum_size.x = 140
		slider.focus_mode = Control.FOCUS_CLICK
		row.add_child(slider)

		var val_label = Label.new()
		val_label.custom_minimum_size.x = 65
		val_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		val_label.add_theme_font_size_override("font_size", 13)
		row.add_child(val_label)

		var prop = param["prop"]
		var target = param.get("target", "car")
		_sliders[prop] = {"slider": slider, "label": val_label, "step": param["step"], "target": target}
		slider.value_changed.connect(_on_slider_changed.bind(prop))

	vbox.add_child(HSeparator.new())
	var btn = Button.new()
	btn.text = "Reset Defaults"
	btn.pressed.connect(_reset_defaults)
	vbox.add_child(btn)

func set_car(new_car: VehicleBody3D):
	car = new_car
	_sync_all()

func set_ros_bridge(bridge: Node):
	ros_bridge = bridge
	_sync_all()

func _get_target(prop: String) -> Node:
	var info = _sliders.get(prop, {})
	if info.get("target") == "bridge" and ros_bridge and is_instance_valid(ros_bridge):
		return ros_bridge
	if car and is_instance_valid(car):
		return car
	return null

func _sync_all():
	for prop in _sliders:
		var node = _get_target(prop)
		if node:
			var val = node.get(prop)
			if val != null:
				_sliders[prop]["slider"].set_value_no_signal(val)
				_fmt(prop, val)

func _on_slider_changed(value: float, prop: String):
	_fmt(prop, value)
	var node = _get_target(prop)
	if not node:
		return
	node.set(prop, value)
	# Car-specific handling
	if _sliders[prop]["target"] == "car" and car and is_instance_valid(car):
		if prop in GEOM_PROPS:
			car_rebuild_requested.emit()
		else:
			car.apply_vehicle_params()

func _fmt(prop: String, value: float):
	var s = _sliders[prop]["step"]
	_sliders[prop]["label"].text = ("%.3f" if s < 0.01 else ("%.2f" if s < 0.1 else ("%.1f" if s < 1.0 else "%.0f"))) % value

func _reset_defaults():
	if not car:
		return
	var fresh = preload("res://car.tscn").instantiate()
	for prop in _sliders:
		if _sliders[prop]["target"] == "car":
			var val = fresh.get(prop)
			if val != null:
				car.set(prop, val)
	fresh.queue_free()
	# Reset sensor delays to 0
	if ros_bridge and is_instance_valid(ros_bridge):
		for prop in _sliders:
			if _sliders[prop]["target"] == "bridge":
				ros_bridge.set(prop, 0.0)
	car_rebuild_requested.emit()
	_sync_all()
