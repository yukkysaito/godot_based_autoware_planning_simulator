# Release Procedure

This document describes the native Linux release flow for `godot_based_autoware_planning_simulator`.

The release package currently contains:

- `godot_autoware_simulator.x86_64`
- `vehicle_params.json`
- `scripts/lanelet_bridge_node.py`

## Prerequisites

- Godot editor binary is built locally
- Linux export template is installed
- `gh` is installed and authenticated
- working tree is clean before tagging

Example Godot binary path used in this repository:

```bash
../godot/bin/godot.linuxbsd.editor.x86_64
```

## 1. Update the app version

Update the release version in [`project.godot`](../project.godot):

```ini
[application]
config/version="X.Y.Z"
```

This value is used by the in-app update notice. If this is not updated, the app may claim an update is available even after a release.

Update [`version.json`](../version.json) at the same time:

```json
{
  "latest_version": "X.Y.Z",
  "release_url": "https://github.com/yukkysaito/godot_based_autoware_planning_simulator/releases/tag/vX.Y.Z",
  "download_url_linux": "https://github.com/yukkysaito/godot_based_autoware_planning_simulator/releases/download/vX.Y.Z/godot_autoware_simulator-vX.Y.Z.tar.gz"
}
```

`version.json` is polled by release binaries every minute. If it is stale, the update notice will be wrong.

## 2. Sanity check before release

Run the editor headless once to catch script errors:

```bash
../godot/bin/godot.linuxbsd.editor.x86_64 --path ./ --headless --quit
```

Expected warning:

- rosbridge connection may fail if `ws://localhost:9090` is not running

That warning does not block the release by itself.

## 3. Build the Linux binary

Export using the existing preset from [`export_presets.cfg`](../export_presets.cfg):

```bash
mkdir -p build
../godot/bin/godot.linuxbsd.editor.x86_64 \
  --path ./ \
  --headless \
  --export-release "Linux/X11" \
  build/godot_autoware_simulator.x86_64
```

Confirm the binary exists:

```bash
ls -lh build/godot_autoware_simulator.x86_64
```

## 4. Assemble the release package

Create a staging directory:

```bash
rm -rf build/release-vX.Y.Z
mkdir -p build/release-vX.Y.Z/scripts
cp build/godot_autoware_simulator.x86_64 build/release-vX.Y.Z/godot_autoware_simulator.x86_64
cp vehicle_params.json build/release-vX.Y.Z/vehicle_params.json
cp scripts/lanelet_bridge_node.py build/release-vX.Y.Z/scripts/lanelet_bridge_node.py
```

Create the tarball:

```bash
tar czf build/godot_autoware_simulator-vX.Y.Z.tar.gz -C build/release-vX.Y.Z .
```

Verify contents:

```bash
tar tzf build/godot_autoware_simulator-vX.Y.Z.tar.gz
```

## 5. Commit the release changes

Typical changes before a release:

- version bump in `project.godot`
- release metadata update in `version.json`
- any final bug fixes
- documentation updates

Commit them normally:

```bash
git status --short
git add project.godot version.json README.md docs/ main.gd ros_bridge.gd tuning_panel.gd traffic_light_manager.gd
git commit -m "Prepare vX.Y.Z release"
```

Only stage files that actually belong to the release.

## 6. Create and push the tag

Make sure `main` is current and clean:

```bash
git status --short --branch
```

Create the annotated tag:

```bash
git tag -a vX.Y.Z -m "vX.Y.Z"
```

Push commit and tag:

```bash
git push origin main
git push origin vX.Y.Z
```

## 7. Create the GitHub release

Create the release and upload the tarball:

```bash
gh release create vX.Y.Z \
  build/godot_autoware_simulator-vX.Y.Z.tar.gz \
  --title "vX.Y.Z" \
  --notes "Release vX.Y.Z"
```

If you want richer notes, write them in a temporary file and use `--notes-file`.

## 8. Verify the published release

Check:

- release page exists
- uploaded asset is present
- tarball downloads correctly
- extracted package contains the expected files
- app starts and reports the new version as current

Useful commands:

```bash
gh release view vX.Y.Z --json url,assets
gh release download vX.Y.Z --pattern 'godot_autoware_simulator-vX.Y.Z.tar.gz' --dir /tmp/release-check
```

## Notes

### Update notice behavior

The app polls [`version.json`](../version.json) once per minute and compares `latest_version` against `application/config/version` in [`project.godot`](../project.godot).

That means:

- always bump `config/version` before releasing
- always update `version.json` in the same release preparation commit
- `release_url` and `download_url_linux` should point at the final published release
- after pushing `main`, create the GitHub release promptly so the URLs in `version.json` become valid

### Export template requirement

The Linux export depends on a local template installation, typically under:

```bash
~/.local/share/godot/export_templates/<godot-version>/
```

If export fails with a missing template error, install or copy the correct Linux release template first.
