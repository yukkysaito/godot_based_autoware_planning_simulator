extends PanelContainer

const VERSION_SETTING := "application/config/version"
const UPDATE_URL_SETTING := "application/config/update_check_url"
const DEFAULT_UPDATE_URL := "https://raw.githubusercontent.com/yukkysaito/godot_based_autoware_planning_simulator/main/version.json"
const CHECK_INTERVAL_SEC := 60.0

var _label: Label
var _download_btn: Button
var _row: HBoxContainer
var _http: HTTPRequest
var _timer: Timer
var _current_version := ""
var _latest_available_version := ""
var _release_url := ""
var _download_url := ""
var _request_in_flight := false

func _ready():
	visible = true
	mouse_filter = Control.MOUSE_FILTER_STOP
	_current_version = str(ProjectSettings.get_setting(VERSION_SETTING, "0.0.0"))
	_build_ui()
	_set_version_only_state()

	_http = HTTPRequest.new()
	_http.timeout = 8.0
	_http.request_completed.connect(_on_request_completed)
	add_child(_http)

	_timer = Timer.new()
	_timer.wait_time = CHECK_INTERVAL_SEC
	_timer.one_shot = false
	_timer.timeout.connect(_check_for_update)
	add_child(_timer)

	if _should_check():
		_timer.start()
		_check_for_update()

func _build_ui():
	custom_minimum_size = Vector2(150, 0)
	var style = StyleBoxFlat.new()
	style.bg_color = Color(0.07, 0.08, 0.11, 0.92)
	style.border_color = Color(0.42, 0.47, 0.58, 0.32)
	style.border_width_left = 1
	style.border_width_top = 1
	style.border_width_right = 1
	style.border_width_bottom = 1
	style.corner_radius_top_left = 12
	style.corner_radius_top_right = 12
	style.corner_radius_bottom_left = 12
	style.corner_radius_bottom_right = 12
	style.shadow_color = Color(0, 0, 0, 0.25)
	style.shadow_size = 6
	style.content_margin_left = 12
	style.content_margin_right = 12
	style.content_margin_top = 8
	style.content_margin_bottom = 8
	add_theme_stylebox_override("panel", style)

	_row = HBoxContainer.new()
	_row.alignment = BoxContainer.ALIGNMENT_CENTER
	_row.add_theme_constant_override("separation", 8)
	add_child(_row)

	_label = Label.new()
	_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
	_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_label.add_theme_font_size_override("font_size", 12)
	_label.add_theme_color_override("font_color", Color(1, 1, 1, 0.92))
	_row.add_child(_label)

	_download_btn = Button.new()
	_download_btn.text = "Update"
	_download_btn.custom_minimum_size = Vector2(58, 0)
	_download_btn.pressed.connect(_open_download)
	_row.add_child(_download_btn)

func _should_check() -> bool:
	if OS.has_feature("dedicated_server"):
		return false
	if OS.has_feature("editor"):
		return false
	return true

func _check_for_update():
	if _request_in_flight:
		return
	var url = str(ProjectSettings.get_setting(UPDATE_URL_SETTING, DEFAULT_UPDATE_URL))
	if url.is_empty():
		return
	var headers = PackedStringArray([
		"Accept: application/json",
		"Cache-Control: no-cache",
		"User-Agent: godot-based-autoware-planning-simulator",
	])
	var err = _http.request(url, headers, HTTPClient.METHOD_GET)
	if err != OK:
		push_warning("[UpdateNotice] Version check request failed: %s" % err)
		return
	_request_in_flight = true

func _on_request_completed(result: int, response_code: int, _headers: PackedStringArray, body: PackedByteArray):
	_request_in_flight = false
	if result != HTTPRequest.RESULT_SUCCESS or response_code < 200 or response_code >= 300:
		push_warning("[UpdateNotice] Version check failed: result=%d code=%d" % [result, response_code])
		return
	var json = JSON.new()
	if json.parse(body.get_string_from_utf8()) != OK or not json.data is Dictionary:
		push_warning("[UpdateNotice] Failed to parse version metadata")
		return
	var release = _release_info_from_json(json.data)
	if release.is_empty():
		return
	_apply_release_info(release)

func _release_info_from_json(data: Dictionary) -> Dictionary:
	var latest = str(data.get("latest_version", ""))
	if latest.is_empty():
		return {}
	var release_url = str(data.get("release_url", ""))
	var download_url = _pick_download_url(data, release_url)
	return {
		"latest_version": latest,
		"release_url": release_url,
		"download_url": download_url,
	}

func _apply_release_info(data: Dictionary):
	var latest = str(data.get("latest_version", ""))
	if latest.is_empty():
		return
	_latest_available_version = latest
	_release_url = str(data.get("release_url", ""))
	_download_url = str(data.get("download_url", _release_url))
	if not _is_newer_version(latest, _current_version):
		_set_version_only_state()
		return
	_set_update_available_state(latest)

func _pick_download_url(data: Dictionary, fallback_url: String) -> String:
	var os_name = OS.get_name().to_lower()
	var platform_key = "download_url_%s" % os_name
	if data.has(platform_key):
		return str(data.get(platform_key, fallback_url))
	if os_name == "macos" and data.has("download_url_macos"):
		return str(data.get("download_url_macos", fallback_url))
	if data.has("download_url"):
		return str(data.get("download_url", fallback_url))
	return fallback_url

func _open_download():
	var url = _download_url if not _download_url.is_empty() else _release_url
	if not url.is_empty():
		OS.shell_open(url)

func _set_version_only_state():
	_label.text = "v%s" % _current_version
	_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	tooltip_text = "Current version: v%s" % _current_version
	_download_btn.disabled = true
	_download_btn.visible = false
	visible = true

func _set_update_available_state(latest: String):
	_label.text = "v%s" % _current_version
	_label.autowrap_mode = TextServer.AUTOWRAP_OFF
	tooltip_text = "New version available: v%s" % latest
	_download_btn.tooltip_text = "Download v%s" % latest
	_download_btn.disabled = _download_url.is_empty() and _release_url.is_empty()
	_download_btn.visible = true
	visible = true

func _is_newer_version(remote: String, local: String) -> bool:
	var a = _parse_semver(remote)
	var b = _parse_semver(local)
	var count = maxi(a.size(), b.size())
	for i in range(count):
		var av = a[i] if i < a.size() else 0
		var bv = b[i] if i < b.size() else 0
		if av == bv:
			continue
		return av > bv
	return false

func _parse_semver(version: String) -> Array[int]:
	var normalized = version.strip_edges()
	if normalized.begins_with("v") or normalized.begins_with("V"):
		normalized = normalized.substr(1)
	normalized = normalized.split("-", false, 1)[0]
	var parts = normalized.split(".")
	var numbers: Array[int] = []
	for part in parts:
		var digits := ""
		for ch in part:
			if ch >= "0" and ch <= "9":
				digits += ch
			else:
				break
		numbers.append(int(digits) if not digits.is_empty() else 0)
	return numbers
