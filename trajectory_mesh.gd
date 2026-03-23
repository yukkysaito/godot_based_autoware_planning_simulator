extends MeshInstance3D
## Visualizes Autoware planning trajectory as a triangle strip.
## Matches godot_rviz2 TrajectoryMesh appearance.

var ros_bridge: Node

@export var trajectory_color: Color = Color(0.0, 0.02, 1.0, 0.8)
@export var strip_width: float = 2.0
@export var distance_fade_max: float = 60.0

var _points: Array = []  # Array of { "pos": Vector3, "quat": Quaternion, "vel": float }
var _dirty: bool = false

func _ready():
	mesh = ArrayMesh.new()
	var mat = StandardMaterial3D.new()
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.vertex_color_use_as_albedo = true
	mat.albedo_color = Color.WHITE
	mat.distance_fade_mode = BaseMaterial3D.DISTANCE_FADE_PIXEL_ALPHA
	mat.distance_fade_max_distance = distance_fade_max
	material_override = mat

func _process(_delta):
	if _dirty:
		_dirty = false
		_rebuild_mesh()

func set_trajectory(points: Array):
	_points = points
	_dirty = true

func _rebuild_mesh():
	var am = mesh as ArrayMesh
	am.clear_surfaces()

	if _points.size() < 2:
		return

	var hw = strip_width / 2.0
	var verts = PackedVector3Array()
	var colors = PackedColorArray()
	var normals = PackedVector3Array()

	for pt in _points:
		var pos: Vector3 = pt["pos"]
		var quat: Quaternion = pt["quat"]

		# Forward direction from quaternion, then perpendicular (right)
		var forward = quat * Vector3.FORWARD
		var right = quat * Vector3.RIGHT

		var left_pt = pos - right * hw
		var right_pt = pos + right * hw

		# Right first, then left — winding order must produce UP-facing normals
		verts.append(right_pt)
		verts.append(left_pt)
		normals.append(Vector3.UP)
		normals.append(Vector3.UP)
		colors.append(trajectory_color)
		colors.append(trajectory_color)

	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_COLOR] = colors
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, arrays)
