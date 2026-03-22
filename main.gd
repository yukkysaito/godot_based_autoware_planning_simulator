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
	_setup_lanelet_map()  # after ros_bridge so viewer offset is available
	_setup_hud()

func _setup_environment():
	var env = Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky = Sky.new()
	var sky_mat = ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.35, 0.55, 0.85)
	sky_mat.sky_horizon_color = Color(0.65, 0.75, 0.85)
	sky_mat.ground_bottom_color = Color(0.2, 0.17, 0.13)
	sky_mat.ground_horizon_color = Color(0.65, 0.75, 0.85)
	sky.sky_material = sky_mat
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.5
	env.tonemap_mode = Environment.TONE_MAPPER_ACES
	env.ssao_enabled = true
	var world_env = WorldEnvironment.new()
	world_env.environment = env
	add_child(world_env)

func _setup_lighting():
	var light = DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	light.light_energy = 1.2
	light.shadow_enabled = true
	light.directional_shadow_max_distance = 200.0
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
	mat.albedo_color = Color(0.25, 0.25, 0.27)
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

func _setup_lanelet_map():
	lanelet_map = Node3D.new()
	lanelet_map.set_script(load("res://lanelet_map.gd"))
	lanelet_map.ros_bridge = ros_bridge
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

func _setup_hud():
	var hud = CanvasLayer.new()
	hud.name = "HUD"
	add_child(hud)

	speed_label = _make_label(Vector2(30, 20), 36, Color.WHITE)
	speed_label.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.7))
	speed_label.add_theme_constant_override("shadow_offset_x", 2)
	speed_label.add_theme_constant_override("shadow_offset_y", 2)
	hud.add_child(speed_label)

	info_label = _make_label(Vector2(30, 70), 16, Color(1, 1, 1, 0.7))
	info_label.text = "WASD: Drive | 1:P 2:R 3:N 4:D | R: Respawn | T: Init Pos | M: Manual/Auto | Tab: Tuning"
	hud.add_child(info_label)

	respawn_label = _make_label(Vector2(30, 100), 20, Color(1.0, 0.8, 0.2))
	respawn_label.visible = false
	hud.add_child(respawn_label)

	tuning_panel = PanelContainer.new()
	tuning_panel.set_script(load("res://tuning_panel.gd"))
	tuning_panel.position = Vector2(30, 130)
	hud.add_child(tuning_panel)
	tuning_panel.set_car(car)
	tuning_panel.set_ros_bridge(ros_bridge)
	tuning_panel.car_rebuild_requested.connect(_on_car_rebuild_requested)

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

func _process(_delta):
	if not car or not is_instance_valid(car):
		return
	var spd = car.get_speed_kmh()
	var gear = car.get_gear_name()
	var mode = "AUTO" if ros_bridge.is_autonomous() else "MANUAL"
	var status = ""
	if not car.has_ground_contact():
		status = " [AIR]"
	elif car.is_colliding:
		status = " [COLLISION]"
	speed_label.text = "%d km/h [%s] %s%s" % [int(spd), gear, mode, status]
	if car.is_colliding:
		speed_label.add_theme_color_override("font_color", Color(1, 0.15, 0.1))
	else:
		speed_label.add_theme_color_override("font_color", Color.WHITE)

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

func _make_label(pos: Vector2, size: int, color: Color) -> Label:
	var l = Label.new()
	l.position = pos
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	return l
