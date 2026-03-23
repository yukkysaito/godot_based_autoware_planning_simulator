extends Node3D
## Main scene: loads track, spawns vehicle, sets up camera/HUD/ROS bridge.

const CarScene = preload("res://car.tscn")

var car: VehicleBody3D
var camera: Camera3D
var speed_label: Label
var info_label: Label
var respawn_label: Label
var tuning_panel: PanelContainer
var telemetry: Control
var control_telemetry: Control
var ros_bridge: Node
var trail: MeshInstance3D
var lanelet_map: Node3D
var traffic_light_manager: Node3D
var trajectory_mesh: MeshInstance3D
var dynamic_object_mesh: MeshInstance3D

@export var spawn_position: Vector3 = Vector3(0, 0, 0)
@export var spawn_height_offset: float = 2.0

# ==========================================================================
# Setup
# ==========================================================================

func _ready():
	_setup_environment()
	_setup_lighting()
	_create_default_ground()
	_spawn_car()
	_setup_trail()
	_setup_camera()
	_setup_ros_bridge()
	_setup_traffic_light_manager()
	_setup_trajectory_mesh()
	_setup_dynamic_object_mesh()
	_setup_lanelet_map()  # after ros_bridge so viewer offset is available
	_setup_hud()

func _setup_environment():
	var env = Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky = Sky.new()
	var sky_mat = ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0, 0, 0, 1)
	sky_mat.sky_horizon_color = Color(0.119363, 0.466633, 0.961925, 1)
	sky_mat.sky_curve = 0.00437356
	sky_mat.ground_bottom_color = Color(0.207843, 0.223529, 0.294118, 1)
	sky_mat.ground_horizon_color = Color(0.119363, 0.466633, 0.961925, 1)
	sky_mat.ground_curve = 0.00233258
	sky.sky_material = sky_mat
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.glow_enabled = true
	env.glow_intensity = 8.0
	var world_env = WorldEnvironment.new()
	world_env.environment = env
	add_child(world_env)

func _setup_lighting():
	var light = DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	light.light_energy = 1.2
	add_child(light)

func _create_default_ground():
	## 1km x 1km flat ground at origin with grid pattern for visual reference.
	var ground_size = 1000.0
	var grid_step = 10.0  # 10m grid

	# Dark ground plane
	var mi = MeshInstance3D.new()
	var plane = PlaneMesh.new()
	plane.size = Vector2(ground_size, ground_size)
	mi.mesh = plane
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0, 0.0745098, 0.137255, 1)
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mi.material_override = mat
	add_child(mi)

	# Grid lines using ImmediateMesh
	var grid_mi = MeshInstance3D.new()
	var grid_mesh = ImmediateMesh.new()
	grid_mi.mesh = grid_mesh
	grid_mi.position.y = 0.01  # slightly above ground
	var grid_mat = StandardMaterial3D.new()
	grid_mat.albedo_color = Color(0.5, 0.5, 0.5, 0.5)
	grid_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	grid_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	grid_mi.material_override = grid_mat

	var half = ground_size / 2.0
	grid_mesh.clear_surfaces()
	grid_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	var i = -half
	while i <= half:
		# Lines along Z
		grid_mesh.surface_add_vertex(Vector3(i, 0, -half))
		grid_mesh.surface_add_vertex(Vector3(i, 0, half))
		# Lines along X
		grid_mesh.surface_add_vertex(Vector3(-half, 0, i))
		grid_mesh.surface_add_vertex(Vector3(half, 0, i))
		i += grid_step
	grid_mesh.surface_end()
	add_child(grid_mi)

	# Origin marker: colored axes (10m each)
	var axes_mi = MeshInstance3D.new()
	var axes_mesh = ImmediateMesh.new()
	axes_mi.mesh = axes_mesh
	axes_mi.position.y = 0.02
	var axes_mat = StandardMaterial3D.new()
	axes_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	axes_mat.vertex_color_use_as_albedo = true
	axes_mi.material_override = axes_mat

	axes_mesh.clear_surfaces()
	axes_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	# X axis (red)
	axes_mesh.surface_set_color(Color.RED)
	axes_mesh.surface_add_vertex(Vector3(0, 0, 0))
	axes_mesh.surface_set_color(Color.RED)
	axes_mesh.surface_add_vertex(Vector3(20, 0, 0))
	# Z axis (blue)
	axes_mesh.surface_set_color(Color.BLUE)
	axes_mesh.surface_add_vertex(Vector3(0, 0, 0))
	axes_mesh.surface_set_color(Color.BLUE)
	axes_mesh.surface_add_vertex(Vector3(0, 0, 20))
	axes_mesh.surface_end()
	add_child(axes_mi)

	# Collision
	var body = StaticBody3D.new()
	var col = CollisionShape3D.new()
	var shape = BoxShape3D.new()
	shape.size = Vector3(ground_size, 0.1, ground_size)
	col.shape = shape
	col.position = Vector3(0, -0.05, 0)
	body.add_child(col)
	var phys_mat = PhysicsMaterial.new()
	phys_mat.bounce = 0.0
	phys_mat.friction = 1.0
	body.physics_material_override = phys_mat
	add_child(body)

func _setup_traffic_light_manager():
	traffic_light_manager = Node3D.new()
	traffic_light_manager.set_script(load("res://traffic_light_manager.gd"))
	traffic_light_manager.ros_bridge = ros_bridge
	ros_bridge.traffic_light_manager = traffic_light_manager
	add_child(traffic_light_manager)

func _setup_trajectory_mesh():
	trajectory_mesh = MeshInstance3D.new()
	trajectory_mesh.set_script(load("res://trajectory_mesh.gd"))
	trajectory_mesh.ros_bridge = ros_bridge
	ros_bridge.trajectory_mesh = trajectory_mesh
	add_child(trajectory_mesh)

func _setup_dynamic_object_mesh():
	dynamic_object_mesh = MeshInstance3D.new()
	dynamic_object_mesh.set_script(load("res://dynamic_object_mesh.gd"))
	dynamic_object_mesh.ros_bridge = ros_bridge
	ros_bridge.dynamic_object_mesh = dynamic_object_mesh
	add_child(dynamic_object_mesh)

func _setup_lanelet_map():
	lanelet_map = Node3D.new()
	lanelet_map.set_script(load("res://lanelet_map.gd"))
	lanelet_map.ros_bridge = ros_bridge
	lanelet_map.traffic_light_manager = traffic_light_manager
	ros_bridge.lanelet_map = lanelet_map  # set reference now that both exist
	lanelet_map.map_loaded.connect(_on_map_loaded)
	add_child(lanelet_map)

func _on_map_loaded():
	print("[Main] Map loaded — unfreezing car")
	if car and is_instance_valid(car):
		car.freeze = false

func _spawn_car():
	car = CarScene.instantiate()
	car.position = spawn_position + Vector3(0, spawn_height_offset, 0)
	car.rotation.y = deg_to_rad(13.0)
	car.freeze = true
	add_child(car)
	car.respawned.connect(_on_car_respawned)
	_deferred_unfreeze.call_deferred()

func _deferred_unfreeze():
	await get_tree().physics_frame
	await get_tree().physics_frame
	if car and is_instance_valid(car):
		car.freeze = false

func _setup_trail():
	trail = MeshInstance3D.new()
	trail.set_script(load("res://trail_renderer.gd"))
	trail.car = car
	add_child(trail)

func _setup_camera():
	camera = Camera3D.new()
	camera.fov = 65.0
	camera.far = 2000.0
	camera.set_script(load("res://follow_camera.gd"))
	add_child(camera)
	camera.target = car

func _setup_ros_bridge():
	ros_bridge = Node.new()
	ros_bridge.set_script(load("res://ros_bridge.gd"))
	ros_bridge.car = car
	# lanelet_map reference is set later in _setup_lanelet_map()
	add_child(ros_bridge)

var _dash_speed: Label
var _dash_unit: Label
var _dash_gear: Label
var _dash_mode: Label
var _dash_status: Label
var _dash_lights: Label
var _dash_panel: PanelContainer

func _setup_hud():
	var hud = CanvasLayer.new()
	hud.name = "HUD"
	add_child(hud)

	# --- Dashboard panel (top-left) ---
	_dash_panel = PanelContainer.new()
	_dash_panel.position = Vector2(20, 16)
	var dash_style = _make_panel_style(Color(0.08, 0.08, 0.1, 0.88), 10)
	dash_style.content_margin_left = 20
	dash_style.content_margin_right = 20
	dash_style.content_margin_top = 12
	dash_style.content_margin_bottom = 12
	_dash_panel.add_theme_stylebox_override("panel", dash_style)
	hud.add_child(_dash_panel)

	var dash_hbox = HBoxContainer.new()
	dash_hbox.add_theme_constant_override("separation", 16)
	_dash_panel.add_child(dash_hbox)

	# Speed
	var speed_vbox = VBoxContainer.new()
	speed_vbox.add_theme_constant_override("separation", -4)
	dash_hbox.add_child(speed_vbox)
	_dash_speed = Label.new()
	_dash_speed.text = "0"
	_dash_speed.add_theme_font_size_override("font_size", 52)
	_dash_speed.add_theme_color_override("font_color", Color.WHITE)
	_dash_speed.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_dash_speed.custom_minimum_size.x = 100
	speed_vbox.add_child(_dash_speed)
	_dash_unit = Label.new()
	_dash_unit.text = "km/h"
	_dash_unit.add_theme_font_size_override("font_size", 13)
	_dash_unit.add_theme_color_override("font_color", Color(1, 1, 1, 0.5))
	_dash_unit.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	speed_vbox.add_child(_dash_unit)

	# Separator
	var sep = VSeparator.new()
	sep.add_theme_constant_override("separation", 8)
	dash_hbox.add_child(sep)

	# Gear + Mode + Status
	var info_vbox = VBoxContainer.new()
	info_vbox.add_theme_constant_override("separation", 4)
	dash_hbox.add_child(info_vbox)

	_dash_gear = Label.new()
	_dash_gear.text = "P"
	_dash_gear.add_theme_font_size_override("font_size", 28)
	_dash_gear.add_theme_color_override("font_color", Color(0.4, 0.85, 1.0))
	info_vbox.add_child(_dash_gear)

	_dash_mode = Label.new()
	_dash_mode.text = "MANUAL"
	_dash_mode.add_theme_font_size_override("font_size", 14)
	_dash_mode.add_theme_color_override("font_color", Color(0.5, 1.0, 0.5))
	info_vbox.add_child(_dash_mode)

	_dash_lights = Label.new()
	_dash_lights.text = ""
	_dash_lights.add_theme_font_size_override("font_size", 16)
	info_vbox.add_child(_dash_lights)

	_dash_status = Label.new()
	_dash_status.text = ""
	_dash_status.add_theme_font_size_override("font_size", 13)
	_dash_status.add_theme_color_override("font_color", Color(1, 0.3, 0.2))
	info_vbox.add_child(_dash_status)

	# --- Hotkey bar (bottom-left) ---
	info_label = Label.new()
	info_label.add_theme_font_size_override("font_size", 13)
	info_label.add_theme_color_override("font_color", Color(1, 1, 1, 0.35))
	info_label.text = "WASD Drive  |  1234 P/R/N/D  |  Q/E Turn  |  H Hazard  |  R Respawn  |  T Origin  |  M Auto/Manual  |  Tab Tuning"
	hud.add_child(info_label)
	# Position at bottom — updated in _update_telemetry_position

	# --- Notification (center-top) ---
	respawn_label = Label.new()
	respawn_label.add_theme_font_size_override("font_size", 18)
	respawn_label.add_theme_color_override("font_color", Color(1.0, 0.85, 0.3))
	respawn_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	respawn_label.visible = false
	hud.add_child(respawn_label)

	# Keep old reference for compatibility
	speed_label = _dash_speed

	# --- Tuning panel ---
	tuning_panel = PanelContainer.new()
	tuning_panel.set_script(load("res://tuning_panel.gd"))
	tuning_panel.position = Vector2(20, 120)
	hud.add_child(tuning_panel)
	tuning_panel.set_car(car)
	tuning_panel.set_ros_bridge(ros_bridge)
	tuning_panel.car_rebuild_requested.connect(_on_car_rebuild_requested)

	# --- Telemetry ---
	telemetry = Control.new()
	telemetry.set_script(load("res://telemetry_graph.gd"))
	hud.add_child(telemetry)
	telemetry.car = car

	control_telemetry = Control.new()
	control_telemetry.set_script(load("res://control_telemetry.gd"))
	hud.add_child(control_telemetry)
	control_telemetry.car = car

	_update_telemetry_position()
	get_tree().root.size_changed.connect(_update_telemetry_position)

# ==========================================================================
# Runtime
# ==========================================================================

func _process(delta):
	if not car or not is_instance_valid(car):
		return
	var spd = car.get_speed_kmh()
	_dash_speed.text = "%d" % int(spd)
	_dash_gear.text = car.get_gear_name()

	var is_auto = ros_bridge.is_autonomous()
	_dash_mode.text = "AUTONOMOUS" if is_auto else "MANUAL"
	_dash_mode.add_theme_color_override("font_color",
		Color(0.3, 0.7, 1.0) if is_auto else Color(0.5, 1.0, 0.5))

	if car.is_colliding:
		_dash_status.text = "COLLISION"
		_dash_speed.add_theme_color_override("font_color", Color(1, 0.2, 0.15))
	elif not car.has_ground_contact():
		_dash_status.text = "AIRBORNE"
		_dash_speed.add_theme_color_override("font_color", Color(1, 0.8, 0.2))
	else:
		_dash_status.text = ""
		_dash_speed.add_theme_color_override("font_color", Color.WHITE)

	if car.hazard_lights:
		_dash_lights.text = "<< HAZARD >>"
		_dash_lights.add_theme_color_override("font_color", Color(1.0, 0.6, 0.1))
	elif car.current_turn_signal == car.TurnSignal.LEFT:
		_dash_lights.text = "<< LEFT"
		_dash_lights.add_theme_color_override("font_color", Color(0.2, 1.0, 0.4))
	elif car.current_turn_signal == car.TurnSignal.RIGHT:
		_dash_lights.text = "RIGHT >>"
		_dash_lights.add_theme_color_override("font_color", Color(0.2, 1.0, 0.4))
	else:
		_dash_lights.text = ""

func _input(event):
	if not event is InputEventKey or not event.pressed:
		return
	match event.keycode:
		KEY_ESCAPE:
			get_tree().quit()
		KEY_TAB:
			_toggle_tuning_panel()
		KEY_M:
			ros_bridge.toggle_control_mode()
			get_viewport().set_input_as_handled()

# ==========================================================================
# Car rebuild (geometry param changes)
# ==========================================================================

func _on_car_rebuild_requested():
	var params := {}
	for prop in tuning_panel._sliders:
		params[prop] = car.get(prop)
	var pos = car.global_position
	var rot_y = car.global_rotation.y

	car.queue_free()
	car = CarScene.instantiate()

	for prop in params:
		car.set(prop, params[prop])

	var half_wb = car.wheel_base / 2.0
	var half_tr = car.tread / 2.0
	car.get_node("WheelFL").position = Vector3(-half_tr, 0, -half_wb)
	car.get_node("WheelFR").position = Vector3(half_tr, 0, -half_wb)
	car.get_node("WheelRL").position = Vector3(-half_tr, 0, half_wb)
	car.get_node("WheelRR").position = Vector3(half_tr, 0, half_wb)

	car.position = pos + Vector3(0, 1.0, 0)
	car.rotation.y = rot_y
	add_child(car)

	_rebind_car_references()
	_show_message("Car rebuilt")

func _rebind_car_references():
	car.respawned.connect(_on_car_respawned)
	camera.target = car
	tuning_panel.set_car(car)
	telemetry.car = car
	control_telemetry.car = car
	trail.car = car
	ros_bridge.car = car

# ==========================================================================
# Helpers
# ==========================================================================

func _toggle_tuning_panel():
	tuning_panel.visible = not tuning_panel.visible
	if car and is_instance_valid(car):
		if tuning_panel.visible:
			car.input_enabled = false
		elif not ros_bridge.is_autonomous():
			car.input_enabled = true
	if not tuning_panel.visible:
		get_viewport().gui_release_focus()
	get_viewport().set_input_as_handled()

func _on_car_respawned():
	_show_message("Respawned")

func _show_message(msg: String):
	respawn_label.text = msg
	respawn_label.visible = true
	get_tree().create_timer(2.0).timeout.connect(
		func(): respawn_label.visible = false)

func _update_telemetry_position():
	var vp = get_viewport().get_visible_rect().size
	telemetry.position = Vector2(vp.x - telemetry.size.x - 10, vp.y - telemetry.size.y - 10)
	control_telemetry.position = Vector2(vp.x - control_telemetry.size.x - 10,
		telemetry.position.y - control_telemetry.size.y - 5)
	# Bottom-left hotkey bar
	info_label.position = Vector2(20, vp.y - 30)
	# Center-top notification
	respawn_label.position = Vector2(vp.x / 2 - 100, 16)

func _make_panel_style(bg_color: Color, radius: int) -> StyleBoxFlat:
	var s = StyleBoxFlat.new()
	s.bg_color = bg_color
	s.corner_radius_top_left = radius
	s.corner_radius_top_right = radius
	s.corner_radius_bottom_left = radius
	s.corner_radius_bottom_right = radius
	return s

func _make_label(pos: Vector2, size: int, color: Color) -> Label:
	var l = Label.new()
	l.position = pos
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	return l
