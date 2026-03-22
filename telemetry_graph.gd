extends Control
## Real-time telemetry: lateral G, longitudinal G, longitudinal jerk.
## 10-second scrolling time-series with auto-scaling Y axis.

var car: VehicleBody3D

const TIME_WINDOW: float = 10.0
const SAMPLE_HZ: float = 60.0           # recording rate (independent of physics fps)
const SAMPLE_DT: float = 1.0 / SAMPLE_HZ
const BUFFER_SIZE: int = int(TIME_WINDOW * SAMPLE_HZ)  # 600 samples = 10 sec

const GRAPH_W: float = 280.0
const GRAPH_H: float = 80.0
const GRAPH_PAD: float = 8.0
const LABEL_W: float = 75.0
const SCALE_W: float = 45.0
const TOTAL_W: float = LABEL_W + GRAPH_W + SCALE_W + 10.0
const TOTAL_H: float = (GRAPH_H + GRAPH_PAD) * 3 + 25.0

## Low-pass filter time constant [s]. Larger = smoother.
const FILTER_TC: float = 0.08

var _lat_g := PackedFloat32Array()
var _lon_g := PackedFloat32Array()
var _lon_jerk := PackedFloat32Array()
var _write_idx: int = 0
var _sample_count: int = 0
var _sample_timer: float = 0.0

# Filtered values (low-pass to remove solver noise)
var _filt_lat_g: float = 0.0
var _filt_lon_g: float = 0.0
var _filt_lon_jerk: float = 0.0

# Previous frame state
var _prev_fwd_speed: float = 0.0
var _prev_lat_speed: float = 0.0
var _prev_lon_accel: float = 0.0

func _ready():
	_lat_g.resize(BUFFER_SIZE)
	_lon_g.resize(BUFFER_SIZE)
	_lon_jerk.resize(BUFFER_SIZE)
	_lat_g.fill(0.0)
	_lon_g.fill(0.0)
	_lon_jerk.fill(0.0)
	custom_minimum_size = Vector2(TOTAL_W, TOTAL_H)
	size = custom_minimum_size
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func _physics_process(delta: float):
	if not car or not is_instance_valid(car) or delta < 0.0001:
		return

	# Compute speed components in body frame
	var vel = car.linear_velocity
	var forward = -car.global_transform.basis.z
	var right = car.global_transform.basis.x
	var fwd_speed = vel.dot(forward)
	var lat_speed = vel.dot(right)

	# Differentiate body-frame speeds for acceleration
	var raw_lon_g = (fwd_speed - _prev_fwd_speed) / delta / 9.81
	var raw_lat_g = (lat_speed - _prev_lat_speed) / delta / 9.81
	var raw_lon_accel = raw_lon_g * 9.81
	var raw_jerk = (raw_lon_accel - _prev_lon_accel) / delta

	_prev_fwd_speed = fwd_speed
	_prev_lat_speed = lat_speed
	_prev_lon_accel = raw_lon_accel

	# Low-pass filter (first-order IIR)
	var alpha = 1.0 - exp(-delta / FILTER_TC)
	_filt_lat_g = lerpf(_filt_lat_g, raw_lat_g, alpha)
	_filt_lon_g = lerpf(_filt_lon_g, raw_lon_g, alpha)
	_filt_lon_jerk = lerpf(_filt_lon_jerk, raw_jerk, alpha)

	# Sample at fixed rate
	_sample_timer += delta
	if _sample_timer >= SAMPLE_DT:
		_sample_timer -= SAMPLE_DT
		_lat_g[_write_idx] = _filt_lat_g
		_lon_g[_write_idx] = _filt_lon_g
		_lon_jerk[_write_idx] = _filt_lon_jerk
		_write_idx = (_write_idx + 1) % BUFFER_SIZE
		_sample_count = mini(_sample_count + 1, BUFFER_SIZE)
		queue_redraw()

func _draw():
	# Rounded background
	var bg = StyleBoxFlat.new()
	bg.bg_color = Color(0.06, 0.06, 0.09, 0.88)
	bg.corner_radius_top_left = 8; bg.corner_radius_top_right = 8
	bg.corner_radius_bottom_left = 8; bg.corner_radius_bottom_right = 8
	draw_style_box(bg, Rect2(Vector2.ZERO, size))
	draw_string(ThemeDB.fallback_font, Vector2(10, 16), "Dynamics", HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color(1, 1, 1, 0.4))

	var y_off = 25.0
	_draw_graph("Lat G", _lat_g, y_off, Color(0.2, 0.8, 1.0), _filt_lat_g, "%.2f G", "G")
	y_off += GRAPH_H + GRAPH_PAD
	_draw_graph("Lon G", _lon_g, y_off, Color(1.0, 0.4, 0.3), _filt_lon_g, "%.2f G", "G")
	y_off += GRAPH_H + GRAPH_PAD
	_draw_graph("Jerk", _lon_jerk, y_off, Color(0.4, 1.0, 0.4), _filt_lon_jerk, "%.1f", "m/s3")

func _draw_graph(title: String, data: PackedFloat32Array, y_off: float,
		color: Color, current: float, fmt: String, unit: String):
	var font = ThemeDB.fallback_font
	var gx = LABEL_W
	var gy = y_off
	var n = mini(_sample_count, BUFFER_SIZE)

	# Compute visible data range for auto-scaling
	var data_min: float = 0.0
	var data_max: float = 0.0
	for i in range(n):
		var idx = (_write_idx - n + i + BUFFER_SIZE) % BUFFER_SIZE
		var v = data[idx]
		data_min = minf(data_min, v)
		data_max = maxf(data_max, v)

	# Symmetric range, minimum ±0.1 to avoid division by zero
	var abs_max = maxf(maxf(absf(data_min), absf(data_max)), 0.1)
	# Round up to nice number
	if abs_max < 0.5:
		abs_max = ceilf(abs_max * 10.0) / 10.0
	elif abs_max < 5.0:
		abs_max = ceilf(abs_max * 2.0) / 2.0
	else:
		abs_max = ceilf(abs_max)

	# Label
	draw_string(font, Vector2(4, gy + 14), title, HORIZONTAL_ALIGNMENT_LEFT, -1, 12, color)
	draw_string(font, Vector2(4, gy + 30), fmt % current, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(1, 1, 1, 0.8))

	# Graph bg
	draw_rect(Rect2(gx, gy, GRAPH_W, GRAPH_H), Color(0.1, 0.1, 0.12))

	# Zero line
	var zero_y = gy + GRAPH_H / 2.0
	draw_line(Vector2(gx, zero_y), Vector2(gx + GRAPH_W, zero_y), Color(1, 1, 1, 0.2))

	# ±half grid
	for r in [0.25, 0.75]:
		var ly = gy + GRAPH_H * r
		draw_line(Vector2(gx, ly), Vector2(gx + GRAPH_W, ly), Color(1, 1, 1, 0.07))

	# Y-axis scale labels (right side)
	var sx = gx + GRAPH_W + 3
	draw_string(font, Vector2(sx, gy + 10), "+%.2f %s" % [abs_max, unit], HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1, 1, 1, 0.5))
	draw_string(font, Vector2(sx, gy + GRAPH_H / 2.0 + 4), "0", HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1, 1, 1, 0.3))
	draw_string(font, Vector2(sx, gy + GRAPH_H), "-%.2f" % abs_max, HORIZONTAL_ALIGNMENT_LEFT, -1, 9, Color(1, 1, 1, 0.5))

	# Data line
	if n < 2:
		return
	var points := PackedVector2Array()
	for i in range(n):
		var idx = (_write_idx - n + i + BUFFER_SIZE) % BUFFER_SIZE
		var x = gx + (float(i) / float(BUFFER_SIZE - 1)) * GRAPH_W
		var val = clampf(data[idx], -abs_max, abs_max)
		var y = zero_y - (val / abs_max) * (GRAPH_H / 2.0)
		points.append(Vector2(x, y))
	draw_polyline(points, color, 1.5, true)
