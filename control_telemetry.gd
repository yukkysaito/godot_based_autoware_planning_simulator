extends Control
## Real-time telemetry for control inputs: throttle/brake and steering.
## Shows both command (target) and actual (current with delay) values.

var car: VehicleBody3D

const TIME_WINDOW: float = 10.0
const SAMPLE_HZ: float = 60.0
const SAMPLE_DT: float = 1.0 / SAMPLE_HZ
const BUFFER_SIZE: int = int(TIME_WINDOW * SAMPLE_HZ)

const GRAPH_W: float = 280.0
const GRAPH_H: float = 60.0
const GRAPH_PAD: float = 6.0
const LABEL_W: float = 75.0
const SCALE_W: float = 45.0
const TOTAL_W: float = LABEL_W + GRAPH_W + SCALE_W + 10.0
const TOTAL_H: float = (GRAPH_H + GRAPH_PAD) * 3 + 25.0

var _throttle_cmd := PackedFloat32Array()
var _throttle_act := PackedFloat32Array()
var _brake_cmd := PackedFloat32Array()
var _brake_act := PackedFloat32Array()
var _steer_cmd := PackedFloat32Array()
var _steer_act := PackedFloat32Array()

var _write_idx: int = 0
var _sample_count: int = 0
var _sample_timer: float = 0.0

# Current values for display
var _cur_throttle_cmd: float = 0.0
var _cur_throttle_act: float = 0.0
var _cur_brake_cmd: float = 0.0
var _cur_brake_act: float = 0.0
var _cur_steer_cmd: float = 0.0
var _cur_steer_act: float = 0.0

func _ready():
	for arr in [_throttle_cmd, _throttle_act, _brake_cmd, _brake_act, _steer_cmd, _steer_act]:
		arr.resize(BUFFER_SIZE)
		arr.fill(0.0)
	custom_minimum_size = Vector2(TOTAL_W, TOTAL_H)
	size = custom_minimum_size
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func _physics_process(delta: float):
	if not car or not is_instance_valid(car):
		return

	# cmd = input command (before delay), act = after transport delay
	_cur_throttle_cmd = car.cmd_throttle
	_cur_brake_cmd = car.cmd_brake
	_cur_steer_cmd = car.cmd_steering
	_cur_throttle_act = car._current_throttle
	_cur_brake_act = car._current_brake_val
	_cur_steer_act = car._current_steering

	_sample_timer += delta
	if _sample_timer >= SAMPLE_DT:
		_sample_timer -= SAMPLE_DT
		_throttle_cmd[_write_idx] = _cur_throttle_cmd
		_throttle_act[_write_idx] = _cur_throttle_act
		_brake_cmd[_write_idx] = _cur_brake_cmd
		_brake_act[_write_idx] = _cur_brake_act
		_steer_cmd[_write_idx] = _cur_steer_cmd
		_steer_act[_write_idx] = _cur_steer_act
		_write_idx = (_write_idx + 1) % BUFFER_SIZE
		_sample_count = mini(_sample_count + 1, BUFFER_SIZE)
		queue_redraw()

func _draw():
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.05, 0.05, 0.08, 0.85))
	draw_string(ThemeDB.fallback_font, Vector2(8, 16), "Controls (10s)",
		HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color(1, 0.9, 0.3))

	var y = 25.0
	_draw_pair("Throttle", _throttle_cmd, _throttle_act, y, 0.0, 1.0,
		Color(0.3, 1.0, 0.3, 0.4), Color(0.3, 1.0, 0.3), _cur_throttle_act, "%.2f")
	y += GRAPH_H + GRAPH_PAD
	_draw_pair("Brake", _brake_cmd, _brake_act, y, 0.0, 1.0,
		Color(1.0, 0.3, 0.3, 0.4), Color(1.0, 0.3, 0.3), _cur_brake_act, "%.2f")
	y += GRAPH_H + GRAPH_PAD
	_draw_pair("Steering", _steer_cmd, _steer_act, y, -1.0, 1.0,
		Color(0.3, 0.6, 1.0, 0.4), Color(0.3, 0.6, 1.0), _cur_steer_act, "%+.2f")

func _draw_pair(title: String, cmd_buf: PackedFloat32Array, act_buf: PackedFloat32Array,
		y_off: float, v_min: float, v_max: float,
		cmd_color: Color, act_color: Color, current: float, fmt: String):
	var font = ThemeDB.fallback_font
	var gx = LABEL_W
	var gy = y_off
	var v_range = v_max - v_min

	# Label
	draw_string(font, Vector2(4, gy + 14), title, HORIZONTAL_ALIGNMENT_LEFT, -1, 12, act_color)
	draw_string(font, Vector2(4, gy + 28), fmt % current, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(1, 1, 1, 0.8))

	# Graph bg
	draw_rect(Rect2(gx, gy, GRAPH_W, GRAPH_H), Color(0.1, 0.1, 0.12))

	# Zero/center line
	var zero_y = gy + GRAPH_H * (v_max / v_range)
	draw_line(Vector2(gx, zero_y), Vector2(gx + GRAPH_W, zero_y), Color(1, 1, 1, 0.15))

	# Scale labels
	var sx = gx + GRAPH_W + 3
	draw_string(font, Vector2(sx, gy + 10), "%.1f" % v_max, HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1, 1, 1, 0.4))
	draw_string(font, Vector2(sx, gy + GRAPH_H), "%.1f" % v_min, HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1, 1, 1, 0.4))

	# Legend
	draw_string(font, Vector2(gx + 2, gy + 10), "cmd", HORIZONTAL_ALIGNMENT_LEFT, -1, 9, cmd_color)
	draw_string(font, Vector2(gx + 28, gy + 10), "act", HORIZONTAL_ALIGNMENT_LEFT, -1, 9, act_color)

	if _sample_count < 2:
		return

	# Draw command (thin, transparent) and actual (solid) lines
	var cmd_pts := PackedVector2Array()
	var act_pts := PackedVector2Array()
	var n = mini(_sample_count, BUFFER_SIZE)
	for i in range(n):
		var idx = (_write_idx - n + i + BUFFER_SIZE) % BUFFER_SIZE
		var x = gx + (float(i) / float(BUFFER_SIZE - 1)) * GRAPH_W
		var cmd_val = clampf(cmd_buf[idx], v_min, v_max)
		var act_val = clampf(act_buf[idx], v_min, v_max)
		var cmd_y = gy + (1.0 - (cmd_val - v_min) / v_range) * GRAPH_H
		var act_y = gy + (1.0 - (act_val - v_min) / v_range) * GRAPH_H
		cmd_pts.append(Vector2(x, cmd_y))
		act_pts.append(Vector2(x, act_y))

	if cmd_pts.size() >= 2:
		draw_polyline(cmd_pts, cmd_color, 1.0, true)
	if act_pts.size() >= 2:
		draw_polyline(act_pts, act_color, 2.0, true)
