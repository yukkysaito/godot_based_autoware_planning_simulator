extends Node3D
## Fetches lanelet geometry from bridge via rosbridge services.
## Flow: get batch count → fetch each batch one by one → build mesh.

var ros_bridge: Node
var traffic_light_manager: Node3D  # traffic_light_manager.gd

@export var road_color: Color = Color(0, 0.0745098, 0.137255, 1)
@export var wall_color: Color = Color(0.15, 0.25, 0.4, 0.6)
@export var wall_height: float = 2.0
@export var marking_color: Color = Color(0.529412, 0.529412, 0.529412, 1)
@export var marking_width: float = 0.05
@export var marking_y_offset: float = 0.02

var _total_batches: int = 0
var _fetched_batches: int = 0
var _all_lanelets: Array = []
var _all_intersection_areas: Array = []
var _all_hatched_road_markings: Array = []
var _all_parking_lots: Array = []
var _all_road_borders: Array = []
var _all_shoulders: Array = []
var _all_road_markings: Array = []
var _all_traffic_light_groups: Array = []
var _built: bool = false
var _fetching: bool = false
var _retry_timer: float = 0.0
var _fetch_timeout: float = 0.0

signal map_loaded

func _process(delta):
	if _built or not ros_bridge:
		return
	if _fetching:
		_fetch_timeout += delta
		if _fetch_timeout > 2.0:
			print("[LaneletMap] Fetch timed out, will retry...")
			_fetching = false
			_fetch_timeout = 0.0
		return
	if not ros_bridge.viewer_offset_valid:
		return
	_retry_timer += delta
	if _retry_timer > 3.0:
		_retry_timer = 0.0
		_start_fetch()

func _start_fetch():
	# Step 1: call batch_count service
	print("[LaneletMap] Requesting batch count...")
	_fetching = true
	_fetch_timeout = 0.0
	ros_bridge._send_json({
		"op": "call_service",
		"service": "/godot/lanelet_batch_count",
		"type": "std_srvs/srv/Trigger",
		"args": {},
		"id": "lanelet_batch_count",
	})

func handle_service_response(id: String, values: Dictionary):
	## Called by ros_bridge when a service response arrives.
	if not _fetching:
		return  # ignore stale responses from timed-out requests
	_fetch_timeout = 0.0
	if id == "lanelet_batch_count":
		var count = int(values.get("message", "0"))
		var success = values.get("success", false)
		if success and count > 0:
			_total_batches = count
			_fetched_batches = 0
			_clear_accumulated_data()
			print("[LaneletMap] %d batches available, fetching..." % count)
			_fetch_next_batch(true)
		else:
			print("[LaneletMap] No batches available yet, retrying...")
			_fetching = false

	elif id == "lanelet_batch":
		var success = values.get("success", false)
		var data_str = values.get("message", "")
		if success and data_str.length() > 0:
			var json = JSON.new()
			if json.parse(data_str) == OK and json.data is Dictionary:
				_ingest_batch(json.data)
		_fetched_batches += 1
		print("[LaneletMap] Batch %d/%d fetched" % [_fetched_batches, _total_batches])

		if _fetched_batches < _total_batches:
			_fetch_next_batch(false)
		else:
			print("[LaneletMap] All batches fetched. Building mesh...")
			_fetching = false
			_build_map()

func _clear_accumulated_data():
	_all_lanelets = []
	_all_intersection_areas = []
	_all_hatched_road_markings = []
	_all_parking_lots = []
	_all_road_borders = []
	_all_shoulders = []
	_all_road_markings = []
	_all_traffic_light_groups = []

func _fetch_next_batch(reset: bool):
	ros_bridge._send_json({
		"op": "call_service",
		"service": "/godot/lanelet_batch",
		"type": "example_interfaces/srv/SetBool",
		"args": {"data": reset},
		"id": "lanelet_batch",
	})

func _ingest_batch(data: Dictionary):
	_all_lanelets.append_array(data.get("lanelets", []))
	_all_intersection_areas.append_array(data.get("intersection_areas", []))
	_all_hatched_road_markings.append_array(data.get("hatched_road_markings", []))
	_all_parking_lots.append_array(data.get("parking_lots", []))
	_all_road_borders.append_array(data.get("road_borders", []))
	_all_shoulders.append_array(data.get("shoulders", []))
	_all_road_markings.append_array(data.get("road_markings", []))
	_all_traffic_light_groups.append_array(data.get("traffic_light_groups", []))

func _build_map():
	_built = true
	var lanelets = _all_lanelets
	var ia_list = _all_intersection_areas
	print("[LaneletMap] Building: %d ll, %d ia, %d hm, %d pk, %d rb, %d sh, %d rm" % [
		lanelets.size(), ia_list.size(), _all_hatched_road_markings.size(),
		_all_parking_lots.size(), _all_road_borders.size(),
		_all_shoulders.size(), _all_road_markings.size()])

	if lanelets.size() == 0 and ia_list.size() == 0:
		print("[LaneletMap] ERROR: no geometry data")
		return

	var all_vertices := PackedVector3Array()
	var all_indices := PackedInt32Array()
	var skipped := 0

	for ll in lanelets:
		var left: Array = ll.get("left", [])
		var right: Array = ll.get("right", [])
		if left.size() < 2 or right.size() < 2:
			skipped += 1
			continue
		_triangulate_strip(_to_godot_points(left), _to_godot_points(right), all_vertices, all_indices)

	var ia_tris := 0
	for ia in ia_list:
		var pts_raw: Array = ia.get("points", [])
		if pts_raw.size() < 3:
			continue
		_triangulate_fan(_to_godot_points(pts_raw), all_vertices, all_indices)
		ia_tris += pts_raw.size() - 2

	for hm in _all_hatched_road_markings:
		var pts_raw: Array = hm.get("points", [])
		if pts_raw.size() < 3:
			continue
		_triangulate_fan(_to_godot_points(pts_raw), all_vertices, all_indices)

	for pk in _all_parking_lots:
		var pts_raw: Array = pk.get("points", [])
		if pts_raw.size() < 3:
			continue
		_triangulate_fan(_to_godot_points(pts_raw), all_vertices, all_indices)

	var sh_tris := 0
	for sh in _all_shoulders:
		var left: Array = sh.get("left", [])
		var right: Array = sh.get("right", [])
		if left.size() < 2 or right.size() < 2:
			continue
		var before = all_indices.size()
		_triangulate_strip(_to_godot_points(left), _to_godot_points(right), all_vertices, all_indices)
		sh_tris += (all_indices.size() - before) / 3

	print("[LaneletMap] Road: %d verts, %d tris" % [all_vertices.size(), all_indices.size() / 3])

	if all_vertices.size() == 0:
		print("[LaneletMap] ERROR: no vertices")
		return

	# Fix winding
	for i in range(0, all_indices.size(), 3):
		var i0 = all_indices[i]; var i1 = all_indices[i+1]; var i2 = all_indices[i+2]
		var n = (all_vertices[i1] - all_vertices[i0]).cross(all_vertices[i2] - all_vertices[i0])
		if n.y < 0:
			all_indices[i+1] = i2; all_indices[i+2] = i1

	var arrays = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = all_vertices
	arrays[Mesh.ARRAY_INDEX] = all_indices
	var mesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	var st = SurfaceTool.new()
	st.create_from(mesh, 0)
	st.generate_normals()
	mesh = st.commit()

	var mi = MeshInstance3D.new()
	mi.mesh = mesh
	var mat = StandardMaterial3D.new()
	mat.render_priority = -3
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = road_color
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mi.material_override = mat
	add_child(mi)
	_add_trimesh_collision(mesh)
	_build_walls()
	_build_road_markings()

	var bb_min = all_vertices[0]; var bb_max = all_vertices[0]
	for v in all_vertices:
		bb_min = Vector3(minf(bb_min.x, v.x), minf(bb_min.y, v.y), minf(bb_min.z, v.z))
		bb_max = Vector3(maxf(bb_max.x, v.x), maxf(bb_max.y, v.y), maxf(bb_max.z, v.z))
	var center = (bb_min + bb_max) / 2.0
	# Build traffic lights if data available
	if _all_traffic_light_groups.size() > 0 and traffic_light_manager:
		var converted_groups = _convert_traffic_light_groups(_all_traffic_light_groups)
		traffic_light_manager.set_map(converted_groups)

	print("[LaneletMap] === MAP LOADED ===")
	print("[LaneletMap]   Center: (%.1f, %.1f, %.1f)" % [center.x, center.y, center.z])
	map_loaded.emit()

# ==========================================================================
# Mesh helpers
# ==========================================================================

func _add_trimesh_collision(mesh: ArrayMesh):
	var shape = mesh.create_trimesh_shape()
	if shape:
		shape.backface_collision = true
		var body = StaticBody3D.new()
		var col = CollisionShape3D.new()
		col.shape = shape
		body.add_child(col)
		var pm = PhysicsMaterial.new()
		pm.bounce = 0.0; pm.friction = 1.0
		body.physics_material_override = pm
		add_child(body)

func _build_walls():
	var borders = _all_road_borders
	if borders.is_empty():
		return
	var wv := PackedVector3Array(); var wn := PackedVector3Array(); var wi := PackedInt32Array()
	for border in borders:
		var pts = _to_godot_points(border.get("points", []))
		if pts.size() < 2: continue
		for i in range(pts.size() - 1):
			var base = wv.size()
			var b0 = pts[i]; var b1 = pts[i+1]
			var t0 = b0 + Vector3(0, wall_height, 0)
			var t1 = b1 + Vector3(0, wall_height, 0)
			var seg = (b1 - b0).normalized()
			var n = Vector3(seg.z, 0, -seg.x)
			wv.append(b0); wv.append(b1); wv.append(t0); wv.append(t1)
			wn.append(n); wn.append(n); wn.append(n); wn.append(n)
			wi.append(base); wi.append(base+1); wi.append(base+2)
			wi.append(base+1); wi.append(base+3); wi.append(base+2)
	if wv.is_empty(): return
	var a = []; a.resize(Mesh.ARRAY_MAX)
	a[Mesh.ARRAY_VERTEX] = wv; a[Mesh.ARRAY_NORMAL] = wn; a[Mesh.ARRAY_INDEX] = wi
	var wm = ArrayMesh.new(); wm.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, a)
	var mi = MeshInstance3D.new(); mi.mesh = wm
	var mat = StandardMaterial3D.new()
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.albedo_color = wall_color
	mat.emission_enabled = true
	mat.emission = Color(0.05, 0.1, 0.2, 1)
	mat.emission_energy_multiplier = 2.0
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mi.material_override = mat
	add_child(mi)
	_add_trimesh_collision(wm)
	print("[LaneletMap] Walls: %d borders" % borders.size())

func _build_road_markings():
	var markings = _all_road_markings
	if markings.is_empty(): return
	var mv := PackedVector3Array(); var mi_arr := PackedInt32Array()
	var hw = marking_width / 2.0
	for m in markings:
		var pts = _to_godot_points(m.get("points", []))
		if pts.size() < 2: continue
		for i in range(pts.size() - 1):
			var p0 = pts[i]; var p1 = pts[i+1]
			var seg = (p1 - p0).normalized()
			var perp = Vector3(seg.z, 0, -seg.x) * hw
			var yo = Vector3(0, marking_y_offset, 0)
			var base = mv.size()
			mv.append(p0-perp+yo); mv.append(p0+perp+yo)
			mv.append(p1-perp+yo); mv.append(p1+perp+yo)
			mi_arr.append(base); mi_arr.append(base+1); mi_arr.append(base+2)
			mi_arr.append(base+1); mi_arr.append(base+3); mi_arr.append(base+2)
	if mv.is_empty(): return
	for i in range(0, mi_arr.size(), 3):
		var i0=mi_arr[i]; var i1=mi_arr[i+1]; var i2=mi_arr[i+2]
		var n = (mv[i1]-mv[i0]).cross(mv[i2]-mv[i0])
		if n.y < 0: mi_arr[i+1]=i2; mi_arr[i+2]=i1
	var a = []; a.resize(Mesh.ARRAY_MAX)
	a[Mesh.ARRAY_VERTEX] = mv; a[Mesh.ARRAY_INDEX] = mi_arr
	var mm = ArrayMesh.new(); mm.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, a)
	var st = SurfaceTool.new(); st.create_from(mm, 0); st.generate_normals(); mm = st.commit()
	var mi = MeshInstance3D.new(); mi.mesh = mm
	var mat = StandardMaterial3D.new()
	mat.render_priority = -2
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = marking_color
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat.proximity_fade_distance = 1000.0
	mat.distance_fade_max_distance = 200.0
	mi.material_override = mat
	add_child(mi)
	print("[LaneletMap] Markings: %d lines" % markings.size())

func _to_godot_points(pts: Array) -> PackedVector3Array:
	var result = PackedVector3Array()
	if ros_bridge and ros_bridge.viewer_offset_valid:
		for p in pts:
			result.append(ros_bridge.ros_map_to_godot(float(p[0]), float(p[1]), float(p[2])))
	else:
		for p in pts:
			result.append(Vector3(-float(p[1]), float(p[2]), -float(p[0])))
	return result

func _convert_traffic_light_groups(groups: Array) -> Array:
	## Convert ROS-frame traffic light group data to Godot-frame.
	var result: Array = []
	for g in groups:
		if typeof(g) != TYPE_DICTIONARY:
			continue
		var gid = g.get("group_id", -1)
		var tls_raw = g.get("traffic_lights", [])
		if typeof(tls_raw) != TYPE_ARRAY:
			continue
		var tls_converted: Array = []
		for tl in tls_raw:
			if typeof(tl) != TYPE_DICTIONARY:
				continue
			var converted_tl: Dictionary = {}
			# Convert board (center + normal only; width/height are scalars in meters)
			if tl.has("board") and typeof(tl["board"]) == TYPE_DICTIONARY:
				var board = tl["board"]
				var cb: Dictionary = {}
				if board.has("center"):
					cb["position"] = _ros_point_to_godot(board["center"])
				if board.has("normal"):
					cb["normal"] = _ros_normal_to_godot(board["normal"])
				cb["width"] = float(board.get("width", 0.4))
				cb["height"] = float(board.get("height", 0.9))
				converted_tl["board"] = cb
			# Convert bulbs
			if tl.has("light_bulbs") and typeof(tl["light_bulbs"]) == TYPE_ARRAY:
				var bulbs_out: Array = []
				for bulb in tl["light_bulbs"]:
					if typeof(bulb) != TYPE_DICTIONARY:
						continue
					var cb_bulb: Dictionary = {}
					if bulb.has("position"):
						cb_bulb["position"] = _ros_point_to_godot(bulb["position"])
					cb_bulb["radius"] = float(bulb.get("radius", 0.05))
					if bulb.has("normal"):
						cb_bulb["normal"] = _ros_normal_to_godot(bulb["normal"])
					else:
						cb_bulb["normal"] = Vector3.FORWARD
					cb_bulb["color"] = str(bulb.get("color", "none"))
					cb_bulb["arrow"] = str(bulb.get("arrow", "none"))
					bulbs_out.append(cb_bulb)
				converted_tl["light_bulbs"] = bulbs_out
			tls_converted.append(converted_tl)
		result.append({"group_id": gid, "traffic_lights": tls_converted})
	return result

func _ros_point_to_godot(p) -> Vector3:
	## Convert a ROS point (Array [x,y,z] or Dictionary {x,y,z}) to Godot.
	if p is Array and p.size() >= 3:
		return ros_bridge.ros_map_to_godot(float(p[0]), float(p[1]), float(p[2]))
	elif p is Dictionary:
		return ros_bridge.ros_map_to_godot(
			float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("z", 0)))
	return Vector3.ZERO

func _ros_normal_to_godot(n) -> Vector3:
	## Convert a ROS normal direction to Godot frame (rotation only, no offset).
	var rx: float = 0; var ry: float = 0; var rz: float = 0
	if n is Array and n.size() >= 3:
		rx = float(n[0]); ry = float(n[1]); rz = float(n[2])
	elif n is Dictionary:
		rx = float(n.get("x", 0)); ry = float(n.get("y", 0)); rz = float(n.get("z", 0))
	# ROS (x=fwd, y=left, z=up) -> Godot (x=-y, y=z, z=-x)
	return Vector3(-ry, rz, -rx)

func _triangulate_fan(pts: PackedVector3Array, verts: PackedVector3Array, idx: PackedInt32Array):
	if pts.size() < 3: return
	var pts2d := PackedVector2Array()
	for p in pts: pts2d.append(Vector2(p.x, p.z))
	var tri = Geometry2D.triangulate_polygon(pts2d)
	if tri.is_empty():
		pts2d.reverse()
		tri = Geometry2D.triangulate_polygon(pts2d)
		if tri.is_empty(): return
		var rev := PackedVector3Array()
		for i in range(pts.size()-1, -1, -1): rev.append(pts[i])
		pts = rev
	var base = verts.size()
	for p in pts: verts.append(p)
	for ti in tri: idx.append(base + ti)

func _triangulate_strip(left: PackedVector3Array, right: PackedVector3Array,
		verts: PackedVector3Array, idx: PackedInt32Array):
	var ln := left.size(); var rn := right.size()
	if ln < 2 or rn < 2: return
	var ld := PackedFloat32Array(); ld.resize(ln); ld[0] = 0.0
	for i in range(1, ln): ld[i] = ld[i-1] + left[i].distance_to(left[i-1])
	var lt := ld[ln-1] if ld[ln-1] > 0.001 else 1.0
	var rd := PackedFloat32Array(); rd.resize(rn); rd[0] = 0.0
	for i in range(1, rn): rd[i] = rd[i-1] + right[i].distance_to(right[i-1])
	var rt := rd[rn-1] if rd[rn-1] > 0.001 else 1.0
	var bl := verts.size()
	for p in left: verts.append(p)
	var br := verts.size()
	for p in right: verts.append(p)
	var li := 0; var ri := 0
	while li < ln-1 or ri < rn-1:
		var l_next := ld[li+1]/lt if li < ln-1 else 2.0
		var r_next := rd[ri+1]/rt if ri < rn-1 else 2.0
		if l_next <= r_next and li < ln-1:
			idx.append(bl+li); idx.append(br+ri); idx.append(bl+li+1); li += 1
		elif ri < rn-1:
			idx.append(bl+li); idx.append(br+ri); idx.append(br+ri+1); ri += 1
		else: break
