extends Camera3D

var target: Node3D

## Camera distance from target [m]
@export var distance: float = 10.0
@export var min_distance: float = 3.0
@export var max_distance: float = 500.0
## Height offset above car center [m]
@export var height_offset: float = 1.5
## Mouse sensitivity for orbiting
@export var mouse_sensitivity: float = 0.003
## Position smoothing speed
@export var position_smooth: float = 8.0
## Zoom step per scroll tick
## Zoom step per scroll tick (proportional to current distance)
@export var zoom_factor: float = 0.15
## Auto-follow: when not dragging, camera yaw gradually returns behind the car
@export var auto_follow_speed: float = 2.0

var _yaw: float = 0.0       # Horizontal orbit angle (radians)
var _pitch: float = -0.3    # Vertical orbit angle (radians, negative = looking down)
var _is_orbiting: bool = false
var _auto_follow_timer: float = 0.0

func _unhandled_input(event: InputEvent):
	if event is InputEventMouseButton:
		# Skip scroll when mouse is over a GUI control (tuning panel etc.)
		var gui_has_focus = get_viewport().gui_get_hovered_control() != null
		match event.button_index:
			MOUSE_BUTTON_RIGHT:
				_is_orbiting = event.pressed
				if event.pressed:
					Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
					_auto_follow_timer = 0.0
				else:
					Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
			MOUSE_BUTTON_WHEEL_UP:
				if not gui_has_focus:
					distance = maxf(min_distance, distance * (1.0 - zoom_factor))
			MOUSE_BUTTON_WHEEL_DOWN:
				if not gui_has_focus:
					distance = minf(max_distance, distance * (1.0 + zoom_factor))

	elif event is InputEventMouseMotion and _is_orbiting:
		_yaw -= event.relative.x * mouse_sensitivity
		_pitch -= event.relative.y * mouse_sensitivity
		_pitch = clampf(_pitch, -1.2, 0.3)  # ~-69deg to ~17deg
		_auto_follow_timer = 0.0

func _physics_process(delta: float):
	if not target:
		return

	# Auto-follow: smoothly rotate back behind the car when not manually orbiting
	if not _is_orbiting:
		_auto_follow_timer += delta
		if _auto_follow_timer > 1.0:
			# Target yaw: behind the car (car forward is -Z)
			var car_forward = -target.global_transform.basis.z
			var target_yaw = atan2(car_forward.x, car_forward.z) + PI
			# Shortest-path angle lerp
			var diff = fmod(target_yaw - _yaw + 3.0 * PI, TAU) - PI
			_yaw += diff * auto_follow_speed * delta

	# Target position (car center + height offset)
	var pivot = target.global_position + Vector3(0, height_offset, 0)

	# Spherical coordinates -> offset
	var cos_pitch = cos(_pitch)
	var offset = Vector3(
		distance * cos_pitch * sin(_yaw),
		distance * -sin(_pitch),
		distance * cos_pitch * cos(_yaw)
	)

	var desired_pos = pivot + offset
	global_position = global_position.lerp(desired_pos, position_smooth * delta)
	look_at(pivot, Vector3.UP)
