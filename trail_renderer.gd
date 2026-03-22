extends MeshInstance3D
## Renders a 10-second trail of the vehicle's base_link position in 3D.

var car: VehicleBody3D

@export var trail_duration: float = 10.0    ## How long trail remains [s]
@export var sample_interval: float = 0.05   ## Sample every 50ms
@export var trail_width: float = 0.15       ## Line width [m]
@export var trail_color: Color = Color(0.0, 1.0, 0.4, 0.8)
@export var trail_y_offset: float = 0.05    ## Lift above ground [m]

var _points: Array = []  # Array of [timestamp, Vector3]
var _timer: float = 0.0
var _mat: StandardMaterial3D

func _ready():
	_mat = StandardMaterial3D.new()
	_mat.albedo_color = trail_color
	_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	material_override = _mat

func _process(delta):
	if not car or not is_instance_valid(car):
		return

	_timer += delta
	if _timer >= sample_interval:
		_timer -= sample_interval
		# base_link = rear axle center on ground
		var rl = car.get_node("WheelRL").global_position
		var rr = car.get_node("WheelRR").global_position
		var rear_axle = (rl + rr) / 2.0
		var pos = Vector3(rear_axle.x, rear_axle.y - car.wheel_radius_param + trail_y_offset, rear_axle.z)
		var now = Time.get_ticks_msec() / 1000.0
		_points.append([now, pos])

	# Remove old points
	var now = Time.get_ticks_msec() / 1000.0
	while _points.size() > 0 and _points[0][0] < now - trail_duration:
		_points.pop_front()

	_rebuild_mesh()

func _rebuild_mesh():
	if _points.size() < 2:
		mesh = null
		return

	var im = ImmediateMesh.new()
	im.surface_begin(Mesh.PRIMITIVE_TRIANGLE_STRIP)

	var now = Time.get_ticks_msec() / 1000.0
	for i in range(_points.size()):
		var t = _points[i][0]
		var pos = _points[i][1]

		# Fade alpha based on age
		var age_ratio = clampf((now - t) / trail_duration, 0.0, 1.0)
		var alpha = (1.0 - age_ratio) * trail_color.a

		# Direction to next point (or from previous)
		var dir: Vector3
		if i < _points.size() - 1:
			dir = (_points[i + 1][1] - pos).normalized()
		elif i > 0:
			dir = (pos - _points[i - 1][1]).normalized()
		else:
			continue

		# Perpendicular in XZ plane
		var perp = Vector3(dir.z, 0, -dir.x) * (trail_width / 2.0)

		im.surface_set_color(Color(trail_color.r, trail_color.g, trail_color.b, alpha))
		im.surface_add_vertex(pos - perp)
		im.surface_set_color(Color(trail_color.r, trail_color.g, trail_color.b, alpha))
		im.surface_add_vertex(pos + perp)

	im.surface_end()
	mesh = im
