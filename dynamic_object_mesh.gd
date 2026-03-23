extends MeshInstance3D
## Visualizes perception dynamic objects as 3D polygon meshes.
## Subscribes to /perception/object_recognition/objects (PredictedObjects).

var ros_bridge: Node

var _objects: Array = []
var _dirty: bool = false

func _ready():
	mesh = ArrayMesh.new()
	var mat = StandardMaterial3D.new()
	mat.render_priority = 1
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.depth_draw_mode = BaseMaterial3D.DEPTH_DRAW_OPAQUE_ONLY
	mat.albedo_color = Color(0.317647, 0.670588, 0.878431, 0.847059)
	mat.emission_enabled = true
	mat.emission = Color(0, 0.0588235, 0.0745098, 1)
	mat.emission_energy_multiplier = 8.0
	mat.distance_fade_mode = BaseMaterial3D.DISTANCE_FADE_PIXEL_ALPHA
	mat.distance_fade_max_distance = 25.0
	material_override = mat

func _process(_delta):
	if _dirty:
		_dirty = false
		_rebuild_mesh()

func set_objects(objects: Array):
	_objects = objects
	_dirty = true

func _rebuild_mesh():
	var am = mesh as ArrayMesh
	am.clear_surfaces()

	var all_verts = PackedVector3Array()
	var all_normals = PackedVector3Array()

	for obj in _objects:
		var pos: Vector3 = obj["position"]
		var quat: Quaternion = obj["quaternion"]
		var shape_type: int = obj.get("shape_type", 0)
		var dims: Vector3 = obj.get("dimensions", Vector3.ONE)
		var footprint: Array = obj.get("footprint", [])

		var basis = Basis(quat)
		var xform = Transform3D(basis, pos)

		match shape_type:
			0:  # BOUNDING_BOX
				_generate_box(xform, dims, all_verts, all_normals)
			1:  # CYLINDER
				_generate_cylinder(xform, dims.x / 2.0, dims.z, all_verts, all_normals)
			2:  # POLYGON
				if footprint.size() >= 3:
					_generate_polygon(xform, footprint, dims.z, all_verts, all_normals)
				else:
					_generate_box(xform, dims, all_verts, all_normals)
			_:
				_generate_box(xform, dims, all_verts, all_normals)

	if all_verts.is_empty():
		return

	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = all_verts
	arrays[Mesh.ARRAY_NORMAL] = all_normals
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

func _generate_box(xform: Transform3D, dims: Vector3,
		verts: PackedVector3Array, norms: PackedVector3Array):
	# dims from ROS: x=length, y=width, z=height
	# C++ maps to body frame: (±dims.y/2, ±dims.x/2, ±dims.z/2)
	# ros2_to_godot body: Godot X=±dims.y/2, Y=±dims.z/2, Z=±dims.x/2
	var hw = dims.y / 2.0  # half-width → Godot X
	var hh = dims.z / 2.0  # half-height → Godot Y
	var hl = dims.x / 2.0  # half-length → Godot Z

	var corners = [
		Vector3(-hw, -hh, -hl), Vector3( hw, -hh, -hl),
		Vector3( hw, -hh,  hl), Vector3(-hw, -hh,  hl),
		Vector3(-hw,  hh, -hl), Vector3( hw,  hh, -hl),
		Vector3( hw,  hh,  hl), Vector3(-hw,  hh,  hl),
	]

	# 6 faces, each as 2 triangles (CCW from outside)
	var faces = [
		[4, 5, 6, 7],  # top (+Y)
		[3, 2, 1, 0],  # bottom (-Y)
		[0, 1, 5, 4],  # front (-Z)
		[2, 3, 7, 6],  # back (+Z)
		[3, 0, 4, 7],  # left (-X)
		[1, 2, 6, 5],  # right (+X)
	]

	for face in faces:
		var v0 = xform * corners[face[0]]
		var v1 = xform * corners[face[1]]
		var v2 = xform * corners[face[2]]
		var v3 = xform * corners[face[3]]
		var normal = (v2 - v0).cross(v1 - v0).normalized()
		# Triangle 1
		verts.append(v0); verts.append(v1); verts.append(v2)
		norms.append(normal); norms.append(normal); norms.append(normal)
		# Triangle 2
		verts.append(v0); verts.append(v2); verts.append(v3)
		norms.append(normal); norms.append(normal); norms.append(normal)

func _generate_polygon(xform: Transform3D, footprint: Array, height: float,
		verts: PackedVector3Array, norms: PackedVector3Array):
	# footprint: Array of Vector2 (XY in local frame)
	# Ensure clockwise winding
	var pts = footprint.duplicate()
	if not _is_clockwise(pts):
		pts.reverse()

	var hh = height / 2.0

	# Footprint pts are in ROS body XY → Godot body (X, 0, -Y)
	# Top face (fan triangulation)
	var top_normal = Vector3.ZERO
	for i in range(2, pts.size()):
		var v0 = xform * Vector3(pts[0].x, hh, -pts[0].y)
		var v1 = xform * Vector3(pts[i - 1].x, hh, -pts[i - 1].y)
		var v2 = xform * Vector3(pts[i].x, hh, -pts[i].y)
		if top_normal == Vector3.ZERO:
			top_normal = (v2 - v0).cross(v1 - v0).normalized()
		verts.append(v0); verts.append(v1); verts.append(v2)
		norms.append(top_normal); norms.append(top_normal); norms.append(top_normal)

	# Bottom face (reversed winding)
	var bottom_normal = -top_normal
	for i in range(2, pts.size()):
		var v0 = xform * Vector3(pts[0].x, -hh, -pts[0].y)
		var v1 = xform * Vector3(pts[i].x, -hh, -pts[i].y)
		var v2 = xform * Vector3(pts[i - 1].x, -hh, -pts[i - 1].y)
		verts.append(v0); verts.append(v1); verts.append(v2)
		norms.append(bottom_normal); norms.append(bottom_normal); norms.append(bottom_normal)

	# Side faces
	for i in range(pts.size()):
		var j = (i + 1) % pts.size()
		var t0 = xform * Vector3(pts[i].x, hh, -pts[i].y)
		var b0 = xform * Vector3(pts[i].x, -hh, -pts[i].y)
		var b1 = xform * Vector3(pts[j].x, -hh, -pts[j].y)
		var t1 = xform * Vector3(pts[j].x, hh, -pts[j].y)
		var side_normal = (b1 - t0).cross(b0 - t0).normalized()
		verts.append(t0); verts.append(b0); verts.append(b1)
		norms.append(side_normal); norms.append(side_normal); norms.append(side_normal)
		verts.append(t0); verts.append(b1); verts.append(t1)
		norms.append(side_normal); norms.append(side_normal); norms.append(side_normal)

func _generate_cylinder(xform: Transform3D, radius: float, height: float,
		verts: PackedVector3Array, norms: PackedVector3Array):
	# 12-sided cylinder (same as C++ generate_cylinder3d)
	var n_sides = 12
	var pts: Array = []
	for i in range(n_sides):
		var angle = (float(n_sides - i) / float(n_sides)) * TAU + PI / float(n_sides)
		pts.append(Vector2(cos(angle) * radius, sin(angle) * radius))

	# Ensure clockwise winding (same as C++)
	if not _is_clockwise(pts):
		pts.reverse()

	# Reuse polygon extrusion (footprint in ROS body XY → Godot body (X, 0, -Y))
	var hh = height / 2.0

	# Top face
	var top_normal = Vector3.ZERO
	for i in range(2, pts.size()):
		var v0 = xform * Vector3(pts[0].x, hh, -pts[0].y)
		var v1 = xform * Vector3(pts[i - 1].x, hh, -pts[i - 1].y)
		var v2 = xform * Vector3(pts[i].x, hh, -pts[i].y)
		if top_normal == Vector3.ZERO:
			top_normal = (v2 - v0).cross(v1 - v0).normalized()
		verts.append(v0); verts.append(v1); verts.append(v2)
		norms.append(top_normal); norms.append(top_normal); norms.append(top_normal)

	# Bottom face
	var bottom_normal = -top_normal
	for i in range(2, pts.size()):
		var v0 = xform * Vector3(pts[0].x, -hh, -pts[0].y)
		var v1 = xform * Vector3(pts[i].x, -hh, -pts[i].y)
		var v2 = xform * Vector3(pts[i - 1].x, -hh, -pts[i - 1].y)
		verts.append(v0); verts.append(v1); verts.append(v2)
		norms.append(bottom_normal); norms.append(bottom_normal); norms.append(bottom_normal)

	# Side faces
	for i in range(pts.size()):
		var j = (i + 1) % pts.size()
		var t0 = xform * Vector3(pts[i].x, hh, -pts[i].y)
		var b0 = xform * Vector3(pts[i].x, -hh, -pts[i].y)
		var b1 = xform * Vector3(pts[j].x, -hh, -pts[j].y)
		var t1 = xform * Vector3(pts[j].x, hh, -pts[j].y)
		var side_normal = (b1 - t0).cross(b0 - t0).normalized()
		verts.append(t0); verts.append(b0); verts.append(b1)
		norms.append(side_normal); norms.append(side_normal); norms.append(side_normal)
		verts.append(t0); verts.append(b1); verts.append(t1)
		norms.append(side_normal); norms.append(side_normal); norms.append(side_normal)

func _is_clockwise(polygon: Array) -> bool:
	var sum = 0.0
	for i in range(polygon.size()):
		var j = (i + 1) % polygon.size()
		sum += polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y
	return sum < 0.0
