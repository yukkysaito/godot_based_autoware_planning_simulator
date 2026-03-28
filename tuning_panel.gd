extends PanelContainer
## In-game parameter tuning panel. Toggle with Tab key.
## Handles both vehicle params (on car) and sensor delay params (on ros_bridge).

var car: VehicleBody3D
var ros_bridge: Node  # ros_bridge.gd instance
var _sliders: Dictionary = {}
var _path_label: Label
var _export_dialog: FileDialog
var _import_dialog: FileDialog

signal car_rebuild_requested

const GEOM_PROPS = ["wheel_base", "tread", "front_overhang", "rear_overhang", "wheel_radius_param",
	"body_height", "body_width_margin"]

# "target" field: "car" or "bridge" — determines which node the property lives on.
const PARAMS = [
	{"group": "Geometry (rebuilds car)"},
	{"name": "Wheel Base [m]", "prop": "wheel_base", "min": 1.5, "max": 6.0, "step": 0.05, "target": "car"},
	{"name": "Tread [m]", "prop": "tread", "min": 1.0, "max": 2.5, "step": 0.01, "target": "car"},
	{"name": "Front Overhang [m]", "prop": "front_overhang", "min": 0.3, "max": 2.0, "step": 0.05, "target": "car"},
	{"name": "Rear Overhang [m]", "prop": "rear_overhang", "min": 0.3, "max": 2.0, "step": 0.05, "target": "car"},
	{"name": "Wheel Radius [m]", "prop": "wheel_radius_param", "min": 0.2, "max": 0.6, "step": 0.01, "target": "car"},
	{"name": "Body Height [m]", "prop": "body_height", "min": 0.5, "max": 4.0, "step": 0.1, "target": "car"},
	{"group": "Powertrain"},
	{"name": "Max Engine Force [N]", "prop": "max_engine_force", "min": 1000, "max": 20000, "step": 100, "target": "car"},
	{"name": "Max Brake Force", "prop": "max_brake_force", "min": 10, "max": 700, "step": 10, "target": "car"},
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
	{"group": "Control Mapping"},
	{"name": "Full Brake Decel [m/s²]", "prop": "full_brake_decel", "min": 0.1, "max": 5.0, "step": 0.1, "target": "bridge"},
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
	{"group": "Lateral Dynamics"},
	{"name": "Understeer Gradient", "prop": "understeer_gradient", "min": 0.0, "max": 0.2, "step": 0.005, "target": "car"},
	{"name": "Drivetrain Efficiency", "prop": "drivetrain_efficiency", "min": 0.1, "max": 1.0, "step": 0.05, "target": "car"},
]

func _ready():
	visible = false
	_build_ui()

func _build_ui():
	var style = StyleBoxFlat.new()
	style.bg_color = Color(0.06, 0.06, 0.09, 0.92)
	style.corner_radius_top_left = 10
	style.corner_radius_top_right = 10
	style.corner_radius_bottom_left = 10
	style.corner_radius_bottom_right = 10
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
	title.text = "Vehicle Tuning"
	title.add_theme_font_size_override("font_size", 16)
	title.add_theme_color_override("font_color", Color(1, 1, 1, 0.6))
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

	_path_label = Label.new()
	_path_label.add_theme_font_size_override("font_size", 11)
	_path_label.add_theme_color_override("font_color", Color(1, 1, 1, 0.4))
	_path_label.text = ""
	vbox.add_child(_path_label)

	var btn_row = HBoxContainer.new()
	btn_row.add_theme_constant_override("separation", 8)
	vbox.add_child(btn_row)

	var btn = Button.new()
	btn.text = "Reset Defaults"
	btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	btn.pressed.connect(_reset_defaults)
	btn_row.add_child(btn)

	var import_btn = Button.new()
	import_btn.text = "Import JSON"
	import_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	import_btn.pressed.connect(_import_params)
	btn_row.add_child(import_btn)

	var export_btn = Button.new()
	export_btn.text = "Export JSON"
	export_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	export_btn.pressed.connect(_export_params)
	btn_row.add_child(export_btn)

func set_car(new_car: VehicleBody3D):
	car = new_car
	_sync_all()
	_update_path_label()

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
	# Reload from the currently loaded JSON (or code defaults if none)
	if not car.loaded_params_path.is_empty():
		car.load_params_from_json(car.loaded_params_path)
	else:
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

func _update_path_label():
	if _path_label and car:
		var p = car.loaded_params_path
		_path_label.text = p if not p.is_empty() else "(defaults)"

func _import_params():
	if not car:
		return
	if not _import_dialog:
		_import_dialog = FileDialog.new()
		_import_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
		_import_dialog.access = FileDialog.ACCESS_FILESYSTEM
		_import_dialog.add_filter("*.json ; JSON files")
		_import_dialog.file_selected.connect(_on_import_path_selected)
		add_child(_import_dialog)
	var default_path = car.loaded_params_path
	if default_path.is_empty() or default_path.begins_with("res://"):
		default_path = OS.get_executable_path().get_base_dir()
	_import_dialog.current_path = default_path
	_import_dialog.popup_centered(Vector2i(600, 400))

func _on_import_path_selected(path: String):
	if not car:
		return
	if car.load_params_from_json(path):
		car.apply_vehicle_params()
		car_rebuild_requested.emit()
		_sync_all()
		_update_path_label()

func _export_params():
	if not car:
		return
	if not _export_dialog:
		_export_dialog = FileDialog.new()
		_export_dialog.file_mode = FileDialog.FILE_MODE_SAVE_FILE
		_export_dialog.access = FileDialog.ACCESS_FILESYSTEM
		_export_dialog.add_filter("*.json ; JSON files")
		_export_dialog.file_selected.connect(_on_export_path_selected)
		add_child(_export_dialog)
	var default_path = car.loaded_params_path
	if default_path.is_empty() or default_path.begins_with("res://"):
		default_path = OS.get_executable_path().get_base_dir().path_join(car.params_json_name)
	_export_dialog.current_path = default_path
	_export_dialog.popup_centered(Vector2i(600, 400))

func _on_export_path_selected(path: String):
	if not car:
		return
	if car.save_params_to_json(path):
		car.loaded_params_path = path
		_update_path_label()
