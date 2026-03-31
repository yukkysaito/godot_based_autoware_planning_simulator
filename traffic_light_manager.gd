extends Node3D
## Manages traffic light visualization from HDMap groups and perception recognition.
## Adapted from godot_rviz2's TrafficLightGroupsManager / TrafficLightGroupActor / TrafficLightBulb.

var ros_bridge: Node

# Bulb textures keyed by "color:arrow"
var _textures: Dictionary = {}

# group_id(int) -> { "root": Node3D, "bulbs_by_key": { "color:arrow": [bulb_data...] }, "all_bulbs": [bulb_data...] }
var _groups: Dictionary = {}

# group_id(int) -> last recognition update time (msec)
var _last_seen: Dictionary = {}

@export var auto_off_seconds: float = 0.7
@export var board_color: Color = Color(0.1, 0.1, 0.1, 1.0)
@export var emission_power: float = 20.0
@export var lens_tint: Color = Color(0.15, 0.15, 0.15, 1.0)
@export var z_offset_board: float = 0.03
@export var z_offset_bulb: float = 0.00

# 9-slice board mesh resources
var _corner_mesh: Mesh
var _edge_mesh: Mesh
var _center_mesh: Mesh
const BOARD_CORNER_SIZE: float = 0.20
const BOARD_BASE_EDGE_LEN: float = 0.20
const BOARD_BASE_CENTER_SIZE: float = 0.20
const BOARD_SEAM_OVERLAP: float = 0.001

var _actors_root: Node3D

func _ready():
	_load_textures()
	_load_board_meshes()
	_actors_root = Node3D.new()
	_actors_root.name = "ActorsRoot"
	add_child(_actors_root)

func _load_board_meshes():
	var base = "res://traffic_light_board/"
	for pair in [["Corner.tres", "_corner_mesh"], ["Edge.tres", "_edge_mesh"], ["Center.tres", "_center_mesh"]]:
		var path = base + pair[0]
		if ResourceLoader.exists(path):
			set(pair[1], load(path))
		else:
			push_warning("[TrafficLightManager] Board mesh not found: %s" % path)

func _load_textures():
	var base_path = "res://traffic_light_textures/"
	var tex_map = {
		"none:none": "traffic_light_black.png",
		"red:none": "traffic_light_red.png",
		"yellow:none": "traffic_light_yellow.png",
		"green:none": "traffic_light_green.png",
		"green:left": "traffic_light_green_left.png",
		"green:right": "traffic_light_green_right.png",
		"green:up": "traffic_light_green_up.png",
		"green:down": "traffic_light_green_down.png",
	}
	for key in tex_map:
		var path = base_path + tex_map[key]
		if ResourceLoader.exists(path):
			var texture = load(path)
			if texture is Texture2D:
				_textures[key] = texture
			else:
				push_warning("[TrafficLightManager] Failed to load texture resource: %s" % path)
		else:
			push_warning("[TrafficLightManager] Texture not found: %s" % path)

func _process(_delta):
	_auto_turn_off_stale()

# ===========================================================================
# Public API — called by lanelet_map.gd
# ===========================================================================

func set_map(groups: Array):
	## Build traffic light actors from HDMap group data.
	## groups = [ { group_id, traffic_lights: [ { board:{...}, light_bulbs:[...] } ] } ]
	var alive: Dictionary = {}
	var now_ms = Time.get_ticks_msec()

	for g in groups:
		if typeof(g) != TYPE_DICTIONARY:
			continue
		var gid = int(g.get("group_id", -1))
		if gid < 0:
			continue
		alive[gid] = true
		_build_group(gid, g)
		_last_seen[gid] = now_ms

	# Remove groups no longer in the map
	var remove_list: Array[int] = []
	for k in _groups.keys():
		if not alive.has(int(k)):
			remove_list.append(int(k))
	for gid in remove_list:
		_remove_group(gid)

	print("[TrafficLightManager] %d groups built" % alive.size())

func update_recognition(status_list: Array):
	## Apply perception recognition results.
	## status_list = [ { group_id, status_elements: [ { color, arrow } ] } ]
	var now_ms = Time.get_ticks_msec()
	for st in status_list:
		if typeof(st) != TYPE_DICTIONARY:
			continue
		var gid = int(st.get("group_id", -1))
		if gid < 0 or not _groups.has(gid):
			continue

		# Turn all off first
		_set_group_all_off(gid)

		var elems = st.get("status_elements", [])
		if typeof(elems) != TYPE_ARRAY:
			continue

		var group_data = _groups[gid]
		var bulbs_by_key: Dictionary = group_data["bulbs_by_key"]

		for elem in elems:
			if typeof(elem) != TYPE_DICTIONARY:
				continue
			var color = str(elem.get("color", ""))
			var arrow = str(elem.get("arrow", "none"))
			var key = "%s:%s" % [color, arrow]

			if bulbs_by_key.has(key):
				for bulb_data in bulbs_by_key[key]:
					_set_bulb_lit(bulb_data, true)

		_last_seen[gid] = now_ms

# ===========================================================================
# Build
# ===========================================================================

func _build_group(gid: int, group: Dictionary):
	# Remove existing if rebuilding
	if _groups.has(gid):
		_remove_group(gid)

	var root = Node3D.new()
	root.name = "TrafficLightGroup_%d" % gid
	_actors_root.add_child(root)

	var group_data = {
		"root": root,
		"bulbs_by_key": {},
		"all_bulbs": [],
	}

	var tls = group.get("traffic_lights", [])
	if typeof(tls) != TYPE_ARRAY:
		_groups[gid] = group_data
		return

	for i in range(tls.size()):
		var tl = tls[i]
		if typeof(tl) != TYPE_DICTIONARY:
			continue
		_build_one_traffic_light(root, group_data, tl, i)

	_groups[gid] = group_data

	# Start all off
	_set_group_all_off(gid)

func _build_one_traffic_light(root: Node3D, group_data: Dictionary, tl: Dictionary, index: int):
	var tl_root = Node3D.new()
	tl_root.name = "TL_%d" % index
	root.add_child(tl_root)

	# --- Board ---
	if tl.has("board") and typeof(tl["board"]) == TYPE_DICTIONARY:
		_create_board(tl_root, tl["board"] as Dictionary)

	# --- Bulbs ---
	var bulbs = tl.get("light_bulbs", [])
	if typeof(bulbs) != TYPE_ARRAY:
		return

	for bulb_dict in bulbs:
		if typeof(bulb_dict) != TYPE_DICTIONARY:
			continue
		var bulb_data = _create_bulb(tl_root, bulb_dict)
		if bulb_data == null:
			continue

		var color = str(bulb_dict.get("color", "none"))
		var arrow = str(bulb_dict.get("arrow", "none"))
		var key = "%s:%s" % [color, arrow]

		group_data["all_bulbs"].append(bulb_data)
		if not group_data["bulbs_by_key"].has(key):
			group_data["bulbs_by_key"][key] = []
		group_data["bulbs_by_key"][key].append(bulb_data)

func _create_board(parent: Node3D, board: Dictionary):
	var pos: Vector3 = board.get("position", Vector3.ZERO)
	var w: float = float(board.get("width", 0.4))
	var h: float = float(board.get("height", 0.9))
	var normal: Vector3 = (board.get("normal", Vector3.FORWARD) as Vector3).normalized()

	# Shared material for all board pieces
	var mat = StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = board_color
	mat.roughness = 1.0
	mat.metallic = 0.0

	var board_root = Node3D.new()
	board_root.name = "Board"
	board_root.position = pos + normal * z_offset_board
	board_root.basis = _basis_from_normal(normal)
	parent.add_child(board_root)

	if not _corner_mesh or not _edge_mesh or not _center_mesh:
		# Fallback: simple plane if meshes not available
		var mi = MeshInstance3D.new()
		var plane = PlaneMesh.new()
		plane.orientation = PlaneMesh.FACE_Z
		plane.size = Vector2(w, h)
		mi.mesh = plane
		mi.material_override = mat
		board_root.add_child(mi)
		return

	# 9-slice board layout (matches reference TrafficLightBoard.gd)
	var half_w = w * 0.5
	var half_h = h * 0.5
	var c = BOARD_CORNER_SIZE * 0.5
	var mid_w = maxf(w - BOARD_CORNER_SIZE * 2.0, 0.001)
	var mid_h = maxf(h - BOARD_CORNER_SIZE * 2.0, 0.001)
	var w_scale = (mid_w + BOARD_SEAM_OVERLAP * 2.0) / BOARD_BASE_EDGE_LEN
	var h_scale = (mid_h + BOARD_SEAM_OVERLAP * 2.0) / BOARD_BASE_EDGE_LEN

	# Corners (position + rotation, no scale)
	var corner_data = [
		[Vector3(-half_w + c,  half_h - c, 0), 0.0],        # top-left
		[Vector3( half_w - c,  half_h - c, 0), PI * 1.5],   # top-right
		[Vector3(-half_w + c, -half_h + c, 0), PI * 0.5],   # bottom-left
		[Vector3( half_w - c, -half_h + c, 0), PI],          # bottom-right
	]
	for cd in corner_data:
		var mi = MeshInstance3D.new()
		mi.mesh = _corner_mesh
		mi.material_override = mat
		mi.position = cd[0]
		mi.rotation.z = cd[1]
		board_root.add_child(mi)

	# Edges (position + rotation + scale.x)
	var edge_data = [
		[Vector3(0,  half_h - c, 0), 0.0,       w_scale],  # top
		[Vector3(0, -half_h + c, 0), PI,         w_scale],  # bottom
		[Vector3(-half_w + c, 0, 0), PI * 0.5,   h_scale],  # left
		[Vector3( half_w - c, 0, 0), PI * 1.5,   h_scale],  # right
	]
	for ed in edge_data:
		var mi = MeshInstance3D.new()
		mi.mesh = _edge_mesh
		mi.material_override = mat
		mi.position = ed[0]
		mi.rotation.z = ed[1]
		mi.scale.x = ed[2]
		board_root.add_child(mi)

	# Center (position + scale XY)
	var center_mi = MeshInstance3D.new()
	center_mi.mesh = _center_mesh
	center_mi.material_override = mat
	center_mi.position = Vector3.ZERO
	center_mi.scale = Vector3(
		(mid_w + BOARD_SEAM_OVERLAP * 2.0) / BOARD_BASE_CENTER_SIZE,
		(mid_h + BOARD_SEAM_OVERLAP * 2.0) / BOARD_BASE_CENTER_SIZE,
		1.0)
	board_root.add_child(center_mi)

func _create_bulb(parent: Node3D, bulb_dict: Dictionary) -> Dictionary:
	var pos: Vector3 = bulb_dict.get("position", Vector3.ZERO)
	var r: float = float(bulb_dict.get("radius", 0.05))
	var n: Vector3 = (bulb_dict.get("normal", Vector3.FORWARD) as Vector3).normalized()
	var color = str(bulb_dict.get("color", "none"))
	var arrow = str(bulb_dict.get("arrow", "none"))
	var tex_key = "%s:%s" % [color, arrow]

	var d = r * 2.0

	# Bulb root node (position + orientation like reference TrafficLightBulb)
	var bulb_root = Node3D.new()
	bulb_root.name = "Bulb"
	bulb_root.position = pos + n * z_offset_bulb
	bulb_root.basis = _basis_from_normal(n)
	parent.add_child(bulb_root)

	# Lens (always visible, darkened) — local z = 0.000
	var lens = MeshInstance3D.new()
	lens.name = "Lens"
	var lens_mesh = PlaneMesh.new()
	lens_mesh.orientation = PlaneMesh.FACE_Z
	lens_mesh.size = Vector2(d, d)
	lens.mesh = lens_mesh
	lens.position.z = 0.000

	var lens_mat = StandardMaterial3D.new()
	lens_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	lens_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	lens_mat.albedo_color = lens_tint
	if _textures.has(tex_key):
		lens_mat.albedo_texture = _textures[tex_key]
	elif _textures.has("none:none"):
		lens_mat.albedo_texture = _textures["none:none"]
	lens.material_override = lens_mat
	bulb_root.add_child(lens)

	# Glow (visible only when lit) — local z = 0.005 (behind lens; -Z is toward viewer)
	var glow = MeshInstance3D.new()
	glow.name = "Glow"
	var glow_mesh = PlaneMesh.new()
	glow_mesh.orientation = PlaneMesh.FACE_Z
	glow_mesh.size = Vector2(d, d)
	glow.mesh = glow_mesh
	glow.position.z = 0.005

	var glow_mat = StandardMaterial3D.new()
	glow_mat.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	glow_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	glow_mat.emission_enabled = true
	glow_mat.emission_energy_multiplier = emission_power
	if _textures.has(tex_key):
		glow_mat.albedo_texture = _textures[tex_key]
		glow_mat.emission_texture = _textures[tex_key]
	glow.material_override = glow_mat
	glow.visible = false
	bulb_root.add_child(glow)

	return {
		"lens": lens,
		"glow": glow,
		"lens_mat": lens_mat,
		"glow_mat": glow_mat,
	}

# ===========================================================================
# State control
# ===========================================================================

func _set_bulb_lit(bulb_data: Dictionary, on: bool):
	var glow: MeshInstance3D = bulb_data["glow"]
	glow.visible = on

func _set_group_all_off(gid: int):
	if not _groups.has(gid):
		return
	for bulb_data in _groups[gid]["all_bulbs"]:
		_set_bulb_lit(bulb_data, false)

func _auto_turn_off_stale():
	if auto_off_seconds <= 0.0:
		return
	var now_ms = Time.get_ticks_msec()
	var threshold_ms = auto_off_seconds * 1000.0
	for gid in _groups.keys():
		if not _last_seen.has(gid):
			continue
		if (now_ms - _last_seen[gid]) > threshold_ms:
			_set_group_all_off(gid)
			_last_seen[gid] = now_ms

func _remove_group(gid: int):
	if _groups.has(gid):
		var root = _groups[gid]["root"]
		if root and is_instance_valid(root):
			root.queue_free()
		_groups.erase(gid)
		_last_seen.erase(gid)

# ===========================================================================
# Geometry helpers
# ===========================================================================

func _basis_from_normal(n: Vector3) -> Basis:
	## Same as reference: Basis.looking_at makes local -Z face the normal.
	## PlaneMesh FACE_Z visible side is -Z direction.
	var normal = n.normalized()
	var up = Vector3.UP
	if abs(normal.dot(up)) > 0.98:
		up = Vector3.RIGHT
	return Basis.looking_at(normal, up)
