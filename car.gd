extends VehicleBody3D
## Configurable vehicle controller with bicycle-model lateral dynamics.
## VehicleBody3D handles suspension/ground contact; this script adds
## realistic cornering forces, weight transfer, and Ackermann approximation.

enum Gear { PARK, REVERSE, NEUTRAL, DRIVE }

# ==========================================================================
# Parameters
# ==========================================================================

@export_group("Geometry")
@export var wheel_base: float = 4.76
@export var rear_overhang: float = 1.0
@export var front_overhang: float = 1.23
@export var tread: float = 1.72
@export var vehicle_weight: float = 8000.0
@export var wheel_radius_param: float = 0.373
@export var body_height: float = 2.2            ## Vehicle box height [m]
@export var body_width_margin: float = 0.36     ## Body width = tread + this [m]

@export_group("Response Delay (transport)")
@export var accel_response_delay: float = 0.6    ## Pure time delay [s]
@export var brake_response_delay: float = 0.12
@export var steering_response_delay: float = 0.45

@export_group("Response Time Constant (1st order lag)")
@export var accel_time_constant: float = 0.15    ## 1st order lag after delay [s]
@export var brake_time_constant: float = 0.05
@export var steering_time_constant: float = 0.1

@export_group("Powertrain")
@export var max_engine_force: float = 10000.0
@export var max_brake_force: float = 70.0
@export var max_steer_angle: float = 0.61
@export var reverse_power_ratio: float = 0.3
@export var steer_speed_threshold: float = 30.0
@export var steer_high_speed_ratio: float = 0.15
@export var creep_force: float = 1500.0
@export var creep_max_speed: float = 8.0

@export_group("Resistance")
@export var rolling_resistance_coeff: float = 0.003
@export var drag_coefficient: float = 0.65
@export var frontal_area: float = 5.4
@export var air_density: float = 1.225
@export var engine_braking_force: float = 20.0

@export_group("Tire Model")
@export var understeer_gradient: float = 0.05            ## Understeer gradient [rad/(m/s²)]. Higher = more understeer at speed

@export_group("Suspension")
@export var suspension_rest_length_val: float = 0.4
@export var suspension_stiffness_val: float = 50.0
@export var suspension_travel_val: float = 0.3
@export var suspension_max_force_val: float = 100000.0
@export var damping_compression_val: float = 4.0
@export var damping_relaxation_val: float = 6.0
@export var wheel_friction_slip: float = 20.0

@export_group("Physics Material")
@export var body_bounce: float = 0.0
@export var body_friction: float = 0.5
@export var center_of_mass_y: float = 0.8        ## Height of CoM from ground [m]

@export_group("Respawn")
@export var respawn_below_y: float = -100.0
@export var flip_respawn_time: float = 3.0

# ==========================================================================
# Runtime state
# ==========================================================================

enum TurnSignal { OFF, LEFT, RIGHT }

var current_gear: Gear = Gear.PARK
var current_turn_signal: TurnSignal = TurnSignal.OFF
var hazard_lights: bool = false
var input_enabled: bool = true
var is_colliding: bool = false

# Unified command input — set by keyboard (manual) or ros_bridge (auto)
var cmd_throttle: float = 0.0     ## 0-1 throttle command
var cmd_brake: float = 0.0        ## 0-1 brake command
var cmd_steering: float = 0.0     ## -1 to +1 steering command

# After transport delay
var _current_throttle: float = 0.0
var _current_brake_val: float = 0.0
var _current_steering: float = 0.0

# Transport delay ring buffers
var _throttle_delay_buf: Array = []
var _brake_delay_buf: Array = []
var _steering_delay_buf: Array = []
var _physics_time: float = 0.0
var _spawn_transform: Transform3D
var _last_good_transform: Transform3D
var _flip_timer: float = 0.0
var _good_pos_timer: float = 0.0
var _contact_count: int = 0

# Lamp references
var _brake_lamp_l: MeshInstance3D
var _brake_lamp_r: MeshInstance3D
var _turn_lamp_fl: MeshInstance3D
var _turn_lamp_fr: MeshInstance3D
var _turn_lamp_rl: MeshInstance3D
var _turn_lamp_rr: MeshInstance3D

signal respawned
signal gear_changed(gear: Gear)

const GEAR_NAMES = {
	Gear.PARK: "P", Gear.REVERSE: "R",
	Gear.NEUTRAL: "N", Gear.DRIVE: "D",
}

# ==========================================================================
# Lifecycle
# ==========================================================================

func _ready():
	apply_vehicle_params()
	contact_monitor = true
	max_contacts_reported = 4
	body_entered.connect(_on_body_entered)
	body_exited.connect(_on_body_exited)
	await get_tree().physics_frame
	_spawn_transform = global_transform
	_last_good_transform = global_transform

func _physics_process(delta):
	if input_enabled:
		_read_keyboard_input()
	# Common path: apply transport delay then set vehicle controls
	_apply_delayed_controls(delta)
	_apply_resistance_forces()
	_update_lamps()
	_record_good_position(delta)
	_check_auto_respawn(delta)

# ==========================================================================
# Vehicle parameter application
# ==========================================================================

func apply_vehicle_params():
	mass = vehicle_weight
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	# Convert ground-based CoM height to local coords (wheel axis = body origin)
	# Ground is at approximately -(wheel_radius + suspension_rest_length) below body origin
	var ground_to_axis = wheel_radius_param + suspension_rest_length_val * 0.9
	center_of_mass = Vector3(0, center_of_mass_y - ground_to_axis, 0)
	continuous_cd = false
	var mat = PhysicsMaterial.new()
	mat.bounce = body_bounce
	mat.friction = body_friction
	physics_material_override = mat
	_apply_wheel_params()
	_build_body_visual()
	_build_wheel_visuals()
	_build_lamps()

func _apply_wheel_params():
	for w in _wheels():
		w.wheel_radius = wheel_radius_param
		w.wheel_rest_length = suspension_rest_length_val
		w.suspension_stiffness = suspension_stiffness_val
		w.suspension_travel = suspension_travel_val
		w.suspension_max_force = suspension_max_force_val
		w.damping_compression = damping_compression_val
		w.damping_relaxation = damping_relaxation_val
		w.wheel_friction_slip = wheel_friction_slip

func _build_body_visual():
	var total_len = front_overhang + wheel_base + rear_overhang
	var body_w = tread + body_width_margin
	var cz = (rear_overhang - front_overhang) / 2.0
	var box_y = body_height / 2.0 + wheel_radius_param + 0.05

	# Collision = same as visual box
	var col: CollisionShape3D = $CollisionShape3D
	var box = BoxShape3D.new()
	box.size = Vector3(body_w, body_height, total_len)
	col.shape = box
	col.position = Vector3(0, box_y, cz)

	# Visual body
	var body_mesh: MeshInstance3D = $BodyMesh
	body_mesh.mesh = _make_box_mesh(body_w, body_height, total_len)
	body_mesh.position = col.position
	if not body_mesh.material_override:
		var bm = _make_material(Color(0.18, 0.22, 0.28, 0.7))
		bm.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		body_mesh.material_override = bm

	# Center of mass marker (red sphere)
	var cabin: MeshInstance3D = $CabinMesh
	cabin.visible = true
	var sphere = SphereMesh.new()
	sphere.radius = 0.15
	sphere.height = 0.3
	sphere.radial_segments = 12
	sphere.rings = 6
	cabin.mesh = sphere
	# Position sphere at CoM in local coords (same as physics center_of_mass)
	cabin.position = center_of_mass
	if not cabin.material_override or not cabin.material_override is StandardMaterial3D:
		var cm_mat = StandardMaterial3D.new()
		cm_mat.albedo_color = Color(1.0, 0.3, 0.2, 0.5)
		cm_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		cabin.material_override = cm_mat

func _build_wheel_visuals():
	for wheel in _wheels():
		# Tire
		var mi := _find_or_create_child_mesh(wheel)
		mi.transform = Transform3D(Basis(Vector3.FORWARD, PI / 2.0), Vector3.ZERO)
		var cyl = CylinderMesh.new()
		cyl.top_radius = wheel_radius_param
		cyl.bottom_radius = wheel_radius_param
		cyl.height = 0.28
		cyl.radial_segments = 16
		mi.mesh = cyl
		if not mi.material_override or mi.material_override.albedo_color != Color(0.1, 0.1, 0.1):
			mi.material_override = _make_material(Color(0.1, 0.1, 0.1))
		# Hub cap (smaller lighter cylinder)
		var hub: MeshInstance3D = null
		for child in wheel.get_children():
			if child is MeshInstance3D and child != mi:
				hub = child
				break
		if hub == null:
			hub = MeshInstance3D.new()
			hub.material_override = _make_material(Color(0.4, 0.4, 0.45))
			wheel.add_child(hub)
		hub.transform = Transform3D(Basis(Vector3.FORWARD, PI / 2.0), Vector3.ZERO)
		var hub_cyl = CylinderMesh.new()
		hub_cyl.top_radius = wheel_radius_param * 0.5
		hub_cyl.bottom_radius = wheel_radius_param * 0.5
		hub_cyl.height = 0.30
		hub_cyl.radial_segments = 12
		hub.mesh = hub_cyl

# ==========================================================================
# Lamps
# ==========================================================================

const _LAMP_OFF_DIM = 0.15
const _LAMP_SIZE = Vector3(0.15, 0.12, 0.05)

func _build_lamps():
	var total_len = front_overhang + wheel_base + rear_overhang
	var body_w = tread + body_width_margin
	var hw = body_w / 2.0
	var hd = total_len / 2.0
	var cz = (rear_overhang - front_overhang) / 2.0
	var lamp_y = wheel_radius_param + body_height * 0.35

	# Rear: brake lamps (red) — at back face, left and right
	_brake_lamp_l = _get_or_create_lamp("BrakeLampL")
	_brake_lamp_l.position = Vector3(-hw * 0.7, lamp_y, cz + hd + 0.01)
	_brake_lamp_r = _get_or_create_lamp("BrakeLampR")
	_brake_lamp_r.position = Vector3(hw * 0.7, lamp_y, cz + hd + 0.01)

	# Rear: turn lamps (amber) — outside of brake lamps
	_turn_lamp_rl = _get_or_create_lamp("TurnLampRL")
	_turn_lamp_rl.position = Vector3(-hw * 0.95, lamp_y, cz + hd + 0.01)
	_turn_lamp_rr = _get_or_create_lamp("TurnLampRR")
	_turn_lamp_rr.position = Vector3(hw * 0.95, lamp_y, cz + hd + 0.01)

	# Front: turn lamps (amber) — at front face corners
	_turn_lamp_fl = _get_or_create_lamp("TurnLampFL")
	_turn_lamp_fl.position = Vector3(-hw * 0.95, lamp_y, cz - hd - 0.01)
	_turn_lamp_fr = _get_or_create_lamp("TurnLampFR")
	_turn_lamp_fr.position = Vector3(hw * 0.95, lamp_y, cz - hd - 0.01)

	# Initialize all off
	_update_lamps()

func _get_or_create_lamp(lamp_name: String) -> MeshInstance3D:
	var existing = get_node_or_null(lamp_name)
	if existing and existing is MeshInstance3D:
		return existing
	var mi = MeshInstance3D.new()
	mi.name = lamp_name
	var bm = BoxMesh.new()
	bm.size = _LAMP_SIZE
	mi.mesh = bm
	var mat = StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.emission_enabled = true
	mat.emission_energy_multiplier = 0.0
	mi.material_override = mat
	add_child(mi)
	return mi

func _set_lamp(lamp: MeshInstance3D, color: Color, on: bool):
	if not lamp or not lamp.material_override:
		return
	var mat: StandardMaterial3D = lamp.material_override
	if on:
		mat.albedo_color = color
		mat.emission = color
		mat.emission_energy_multiplier = 3.0
	else:
		mat.albedo_color = color * _LAMP_OFF_DIM
		mat.emission_energy_multiplier = 0.0

func _update_lamps():
	# Brake: on when brake is pressed
	var braking = _current_brake_val > 0.05
	_set_lamp(_brake_lamp_l, Color(1, 0, 0), braking)
	_set_lamp(_brake_lamp_r, Color(1, 0, 0), braking)

	# Turn signals / hazard
	var left_on = hazard_lights or current_turn_signal == TurnSignal.LEFT
	var right_on = hazard_lights or current_turn_signal == TurnSignal.RIGHT
	var amber = Color(1.0, 0.6, 0.0)
	_set_lamp(_turn_lamp_fl, amber, left_on)
	_set_lamp(_turn_lamp_rl, amber, left_on)
	_set_lamp(_turn_lamp_fr, amber, right_on)
	_set_lamp(_turn_lamp_rr, amber, right_on)

# ==========================================================================
# Queries
# ==========================================================================

func get_gear_name() -> String:
	return GEAR_NAMES.get(current_gear, "?")

func get_forward_speed() -> float:
	return linear_velocity.dot(-global_transform.basis.z)

func get_speed_kmh() -> float:
	return absf(get_forward_speed()) * 3.6

func is_flipped() -> bool:
	return global_transform.basis.y.dot(Vector3.UP) < 0.3

func has_ground_contact() -> bool:
	for w in _wheels():
		if w.is_in_contact():
			return true
	return false

func set_gear(g: Gear):
	if g == current_gear:
		return
	if g != Gear.NEUTRAL and absf(get_forward_speed()) > 3.0:
		return
	current_gear = g
	gear_changed.emit(g)

# ==========================================================================
# Manual input
# ==========================================================================

func _unhandled_key_input(event: InputEvent):
	if not input_enabled or not event.pressed or event.echo:
		return
	match event.keycode:
		KEY_Q:  # Left turn signal toggle
			if current_turn_signal == TurnSignal.LEFT:
				current_turn_signal = TurnSignal.OFF
			else:
				current_turn_signal = TurnSignal.LEFT
				hazard_lights = false
		KEY_E:  # Right turn signal toggle
			if current_turn_signal == TurnSignal.RIGHT:
				current_turn_signal = TurnSignal.OFF
			else:
				current_turn_signal = TurnSignal.RIGHT
				hazard_lights = false
		KEY_H:  # Hazard toggle
			hazard_lights = not hazard_lights
			if hazard_lights:
				current_turn_signal = TurnSignal.OFF

func _read_keyboard_input():
	## Read keyboard and write to cmd_* variables.
	if Input.is_key_pressed(KEY_1): set_gear(Gear.PARK)
	elif Input.is_key_pressed(KEY_2): set_gear(Gear.REVERSE)
	elif Input.is_key_pressed(KEY_3): set_gear(Gear.NEUTRAL)
	elif Input.is_key_pressed(KEY_4): set_gear(Gear.DRIVE)

	var accel_pressed = Input.is_key_pressed(KEY_W) or Input.is_key_pressed(KEY_UP)
	var brake_pressed = Input.is_key_pressed(KEY_S) or Input.is_key_pressed(KEY_DOWN)

	cmd_throttle = 0.0
	cmd_brake = 0.0
	match current_gear:
		Gear.DRIVE:
			if accel_pressed: cmd_throttle = 1.0
			if brake_pressed: cmd_brake = 1.0
		Gear.REVERSE:
			if accel_pressed: cmd_throttle = -reverse_power_ratio
			if brake_pressed: cmd_brake = 1.0
		Gear.PARK:
			cmd_brake = 1.0
		Gear.NEUTRAL:
			if brake_pressed: cmd_brake = 1.0

	cmd_steering = 0.0
	if Input.is_key_pressed(KEY_A) or Input.is_key_pressed(KEY_LEFT): cmd_steering = 1.0
	if Input.is_key_pressed(KEY_D) or Input.is_key_pressed(KEY_RIGHT): cmd_steering = -1.0

	if Input.is_key_pressed(KEY_R): respawn()
	if Input.is_key_pressed(KEY_T): respawn_initial()

func _apply_delayed_controls(delta):
	## Common path: transport delay → 1st order lag → vehicle controls.
	_physics_time += delta
	# Stage 1: pure transport delay
	var delayed_t = _delay_value(_throttle_delay_buf, cmd_throttle, accel_response_delay)
	var delayed_b = _delay_value(_brake_delay_buf, cmd_brake, brake_response_delay)
	var delayed_s = _delay_value(_steering_delay_buf, cmd_steering, steering_response_delay)
	# Stage 2: 1st order lag (exponential smoothing)
	_current_throttle = _first_order_lag(_current_throttle, delayed_t, accel_time_constant, delta)
	_current_brake_val = _first_order_lag(_current_brake_val, delayed_b, brake_time_constant, delta)
	_current_steering = _first_order_lag(_current_steering, delayed_s, steering_time_constant, delta)

	# Apply to VehicleBody3D
	var sr = clampf(1.0 - (get_speed_kmh() - steer_speed_threshold) / 100.0,
					steer_high_speed_ratio, 1.0)
	var raw_steer = _current_steering * max_steer_angle * sr
	# Understeer gradient: δ_eff = δ / (1 + K_us * v² / L)
	# Models speed-dependent steering loss from tire/suspension compliance.
	var spd = absf(get_forward_speed())
	var us_factor = 1.0 / (1.0 + understeer_gradient * spd * spd / wheel_base)
	steering = raw_steer * us_factor
	brake = _current_brake_val * max_brake_force
	# Apply driving force directly to rigid body (bypasses VehicleBody3D tire
	# transmission losses so that F=ma maps accurately to actual acceleration).
	engine_force = 0.0
	var forward = -global_transform.basis.z
	var drive_force = _current_throttle * max_engine_force * 2.0  # 2 traction wheels
	apply_central_force(forward * drive_force)

	# Creep in D/R when idle
	if _current_throttle < 0.05 and _current_brake_val < 0.05:
		var creep_spd = absf(get_forward_speed()) * 3.6
		if creep_spd < creep_max_speed:
			var r = 1.0 - creep_spd / creep_max_speed
			var creep = creep_force * r
			match current_gear:
				Gear.DRIVE:   apply_central_force(forward * creep)
				Gear.REVERSE: apply_central_force(-forward * creep)

# ==========================================================================
# Resistance forces
# ==========================================================================

## Returns the current resistance force [N] at the given speed.
## Used by both the physics step and ros_bridge for throttle feedforward.
func get_resistance_force(speed: float) -> float:
	var f = rolling_resistance_coeff * mass * 9.81
	f += 0.5 * air_density * drag_coefficient * frontal_area * speed * speed
	return f

func _apply_resistance_forces():
	var speed = linear_velocity.length()
	if speed < 0.05:
		return
	var dir = linear_velocity / speed
	var f = get_resistance_force(speed)
	if absf(_current_throttle) < 0.05 and current_gear != Gear.NEUTRAL:
		f += engine_braking_force * clampf(speed / 10.0, 0.1, 1.0)
	apply_central_force(-dir * f)

# ==========================================================================
# Respawn
# ==========================================================================

func respawn():
	_respawn_near_road()

func respawn_initial():
	global_transform = _spawn_transform
	global_position.y += 2.0
	_finalize_respawn()

func _respawn_near_road():
	var yaw = global_rotation.y
	var space = get_world_3d().direct_space_state
	var placed := false
	if space:
		placed = _raycast_place(space, global_position, yaw)
		if not placed:
			var good_yaw = _last_good_transform.basis.get_euler().y
			placed = _raycast_place(space, _last_good_transform.origin, good_yaw)
	if not placed:
		global_transform = _last_good_transform
		global_position.y += 2.0
		global_rotation = Vector3(0, _last_good_transform.basis.get_euler().y, 0)
	_finalize_respawn()

func _raycast_place(space: PhysicsDirectSpaceState3D, pos: Vector3, yaw: float) -> bool:
	var query = PhysicsRayQueryParameters3D.create(
		Vector3(pos.x, pos.y + 100, pos.z),
		Vector3(pos.x, pos.y - 100, pos.z))
	query.collide_with_bodies = true
	query.exclude = [get_rid()]
	var result = space.intersect_ray(query)
	if result:
		global_position = result.position + Vector3(0, 2.0, 0)
		global_rotation = Vector3(0, yaw, 0)
		return true
	return false

func _finalize_respawn():
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_current_throttle = 0.0
	_current_brake_val = 0.0
	_current_steering = 0.0
	_flip_timer = 0.0
	freeze = true
	respawned.emit()
	get_tree().create_timer(0.1).timeout.connect(func():
		if is_instance_valid(self): freeze = false)

func _record_good_position(delta):
	_good_pos_timer += delta
	if _good_pos_timer < 0.5:
		return
	_good_pos_timer = 0.0
	if has_ground_contact():
		_last_good_transform = global_transform

func _check_auto_respawn(delta):
	if global_position.y < respawn_below_y:
		respawn()
		return
	if is_flipped():
		_flip_timer += delta
		if _flip_timer >= flip_respawn_time: respawn()
	else:
		_flip_timer = 0.0

# ==========================================================================
# Collision detection
# ==========================================================================

func _on_body_entered(_body: Node):
	_contact_count += 1
	is_colliding = true

func _on_body_exited(_body: Node):
	_contact_count -= 1
	if _contact_count <= 0:
		_contact_count = 0
		is_colliding = false

# ==========================================================================
# Helpers
# ==========================================================================

func _wheels() -> Array[VehicleWheel3D]:
	return [$WheelFL, $WheelFR, $WheelRL, $WheelRR]

func _first_order_lag(current: float, target: float, tc: float, dt: float) -> float:
	## 1st order lag filter: output approaches target with time constant tc.
	if tc <= 0.001:
		return target
	var alpha = 1.0 - exp(-dt / tc)
	return lerpf(current, target, alpha)

func _delay_value(buf: Array, input: float, delay_sec: float) -> float:
	## Pure transport delay using ring buffer.
	## Returns the input value from delay_sec seconds ago.
	buf.append([_physics_time, input])
	# Remove entries older than needed (keep some margin)
	while buf.size() > 1 and buf[0][0] < _physics_time - delay_sec - 0.1:
		buf.pop_front()
	if delay_sec <= 0.001:
		return input
	# Find the value from delay_sec ago
	var target_time = _physics_time - delay_sec
	if buf[0][0] >= target_time:
		return buf[0][1]  # buffer doesn't go back far enough yet
	# Linear search from oldest
	for i in range(buf.size() - 1):
		if buf[i + 1][0] >= target_time:
			return buf[i][1]  # return value just before target_time
	return input

func _make_box_mesh(w: float, h: float, d: float) -> BoxMesh:
	var m = BoxMesh.new()
	m.size = Vector3(w, h, d)
	return m

func _make_material(color: Color) -> StandardMaterial3D:
	var m = StandardMaterial3D.new()
	m.albedo_color = color
	return m

func _find_or_create_child_mesh(parent: Node) -> MeshInstance3D:
	for child in parent.get_children():
		if child is MeshInstance3D:
			return child
	var mi = MeshInstance3D.new()
	mi.material_override = _make_material(Color(0.15, 0.15, 0.15))
	parent.add_child(mi)
	return mi
