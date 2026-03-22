# Web Build (Browser)

Run the simulator in a web browser using WebAssembly.

## Pre-built

Download `web_build.zip` from [Releases](https://github.com/yukkysaito/godot_based_autoware_planning_simulator/releases), extract it, and serve with the local server below.

## Build from source

Requires [Emscripten](https://emscripten.org/) and Godot export templates for Web.

```bash
# Install Emscripten (if not already)
git clone https://github.com/emscripten-core/emsdk.git ~/emsdk
cd ~/emsdk && ./emsdk install latest && ./emsdk activate latest
source ~/emsdk/emsdk_env.sh

# Build Web export template from Godot source
cd /path/to/godot
scons platform=web target=template_release arch=wasm32 -j$(nproc)

# Install template
TEMPLATE_DIR="$HOME/.local/share/godot/export_templates/$(godot --version | cut -d. -f1-2).dev"
mkdir -p "$TEMPLATE_DIR"
cp bin/godot.web.template_release.wasm32.zip "$TEMPLATE_DIR/web_release.zip"
cp bin/godot.web.template_release.wasm32.zip "$TEMPLATE_DIR/web_nothreads_release.zip"
cp bin/godot.web.template_release.wasm32.zip "$TEMPLATE_DIR/web_debug.zip"
cp bin/godot.web.template_release.wasm32.zip "$TEMPLATE_DIR/web_nothreads_debug.zip"

# Export
mkdir -p build/web
godot --path /path/to/driving_game --headless --export-release "Web" build/web/index.html
```

## Run locally

A standard `python3 -m http.server` will **not** work because Godot Web requires `SharedArrayBuffer`, which needs specific HTTP headers.

Use this server instead:

```bash
cd build/web
python3 -c "
import http.server, socketserver
class H(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        self.send_header('Cross-Origin-Embedder-Policy', 'require-corp')
        super().end_headers()
print('Serving at http://localhost:8060')
socketserver.TCPServer(('', 8060), H).serve_forever()
"
```

Then open **http://localhost:8060** in your browser.

## Connecting to ROS

The browser connects to rosbridge at `ws://localhost:9090` (same as native). Make sure rosbridge is running on the same machine:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml max_message_size:=50000000
```

## Limitations

- Performance is lower than native (especially mesh generation for large maps)
- Requires a modern browser with WebGL2 and SharedArrayBuffer support (Chrome/Edge recommended)
- The lanelet bridge node (`scripts/lanelet_bridge_node.py`) still runs natively — only the simulator UI runs in the browser
