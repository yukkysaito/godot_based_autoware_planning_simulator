#!/usr/bin/env python3
"""ROS2 bridge node: subscribes to /map/vector_map (LaneletMapBin),
deserializes the lanelet2 map, and serves geometry batches on demand
via /godot/lanelet_batch_count and /godot/lanelet_batch services.

Usage:
    source /path/to/autoware/install/setup.bash
    python3 lanelet_bridge_node.py
"""

import ctypes
import json
import os
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from autoware_map_msgs.msg import LaneletMapBin

import lanelet2
from lanelet2.io import Origin, loadRobust
from lanelet2.projection import UtmProjector

# Pre-load Autoware lanelet2 extension to register custom regulatory elements.
# Searches AMENT_PREFIX_PATH and common install paths.
_search_paths = []
for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
    candidate = os.path.join(prefix, "lib", "libautoware_lanelet2_extension_lib.so")
    if os.path.isfile(candidate):
        _search_paths.append(candidate)
for candidate in _search_paths:
    if os.path.isfile(candidate):
        try:
            ctypes.CDLL(candidate, mode=ctypes.RTLD_GLOBAL)
            print(f"[lanelet_bridge] Loaded extension: {candidate}")
            break
        except OSError as e:
            print(f"[lanelet_bridge] WARN: {e}")


class LaneletBridgeNode(Node):
    BATCH_SIZE = 200

    def __init__(self):
        super().__init__("lanelet_bridge_node")

        # Subscribe to vector map
        sub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_ = self.create_subscription(
            LaneletMapBin, "/map/vector_map", self._on_map, sub_qos
        )

        # Services for on-demand batch delivery
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from example_interfaces.srv import SetBool

        self.srv_count_ = self.create_service(
            Trigger, "/godot/lanelet_batch_count", self._on_batch_count
        )
        self.srv_batch_ = self.create_service(
            SetBool, "/godot/lanelet_batch", self._on_batch_request
        )

        self._batches: list = []
        self._last_map_data: bytes = None
        self.get_logger().info("Waiting for /map/vector_map ...")

    def _on_map(self, msg: LaneletMapBin):
        self.get_logger().info(f"Received /map/vector_map ({len(msg.data)} bytes)")
        self._last_map_data = bytes(msg.data)
        self._prepare_batches()

    def _prepare_batches(self):
        lanelet_map = self._deserialize(self._last_map_data)
        if lanelet_map is None:
            return

        all_lanelets = self._extract_lanelets(lanelet_map)
        intersection_areas = self._extract_intersection_areas(lanelet_map)
        road_borders = self._extract_road_borders(lanelet_map)
        shoulders = self._extract_shoulders(lanelet_map)
        road_markings = self._extract_road_markings(lanelet_map)

        self.get_logger().info(
            f"Prepared: {len(all_lanelets)} ll, {len(intersection_areas)} ia, "
            f"{len(road_borders)} rb, {len(shoulders)} sh, {len(road_markings)} rm"
        )

        # Build all batch payloads
        BS = self.BATCH_SIZE
        self._batches = []

        for i in range(0, max(len(all_lanelets), 1), BS):
            self._batches.append({"lanelets": all_lanelets[i:i+BS]})
        for i in range(0, max(len(intersection_areas), 1), BS):
            chunk = intersection_areas[i:i+BS]
            if chunk:
                self._batches.append({"intersection_areas": chunk})
        for i in range(0, max(len(road_borders), 1), BS):
            chunk = road_borders[i:i+BS]
            if chunk:
                self._batches.append({"road_borders": chunk})
        for i in range(0, max(len(shoulders), 1), BS):
            chunk = shoulders[i:i+BS]
            if chunk:
                self._batches.append({"shoulders": chunk})
        for i in range(0, max(len(road_markings), 1), BS):
            chunk = road_markings[i:i+BS]
            if chunk:
                self._batches.append({"road_markings": chunk})

        self.get_logger().info(f"Ready: {len(self._batches)} batches for serving")

    def _on_batch_count(self, request, response):
        """Returns the number of available batches."""
        if not self._batches:
            if self._last_map_data:
                self._prepare_batches()
        response.success = len(self._batches) > 0
        response.message = str(len(self._batches))
        self.get_logger().info(f"Batch count requested → {len(self._batches)}")
        return response

    def _on_batch_request(self, request, response):
        """Returns batch at index (passed via request.data as bool trick:
        the actual index is sent in a separate mechanism).
        We use a simple pop approach: each call returns the next batch."""
        # Use request.data as a reset flag: True = start from beginning
        if request.data:
            self._serve_index = 0

        if not hasattr(self, '_serve_index'):
            self._serve_index = 0

        if self._serve_index < len(self._batches):
            batch = self._batches[self._serve_index]
            response.success = True
            response.message = json.dumps(batch)
            self._serve_index += 1
        else:
            response.success = False
            response.message = ""

        return response

    def _deserialize(self, data: bytes):
        fd, tmp_path = tempfile.mkstemp(suffix=".bin")
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(bytes(data))
            projector = UtmProjector(Origin(0, 0))
            lanelet_map, errors = loadRobust(tmp_path, projector)
            if errors:
                self.get_logger().warn(f"Map loaded with {len(errors)} warnings")
            return lanelet_map
        except Exception as e:
            self.get_logger().error(f"Failed to deserialize: {e}")
            return None
        finally:
            os.unlink(tmp_path)

    def _extract_lanelets(self, lanelet_map) -> list:
        lanelets = []
        for ll in lanelet_map.laneletLayer:
            left_pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ll.leftBound]
            right_pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ll.rightBound]
            if len(left_pts) < 2 or len(right_pts) < 2:
                continue
            lanelets.append({"id": int(ll.id), "left": left_pts, "right": right_pts})
        return lanelets

    def _extract_intersection_areas(self, lanelet_map) -> list:
        areas = []
        for poly in lanelet_map.polygonLayer:
            if "type" not in poly.attributes:
                continue
            if str(poly.attributes["type"]) != "intersection_area":
                continue
            pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in poly]
            if len(pts) < 3:
                continue
            areas.append({"id": int(poly.id), "points": pts})
        return areas

    def _extract_road_borders(self, lanelet_map) -> list:
        borders = []
        for ls in lanelet_map.lineStringLayer:
            try:
                if str(ls.attributes["type"]) != "road_border":
                    continue
            except Exception:
                continue
            pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ls]
            if len(pts) < 2:
                continue
            borders.append({"id": int(ls.id), "points": pts})
        return borders

    def _extract_shoulders(self, lanelet_map) -> list:
        import numpy as np
        from scipy.spatial import KDTree

        boundary_usage = {}
        for ll in lanelet_map.laneletLayer:
            boundary_usage[ll.leftBound.id] = boundary_usage.get(ll.leftBound.id, 0) + 1
            boundary_usage[ll.rightBound.id] = boundary_usage.get(ll.rightBound.id, 0) + 1

        rb_pts_xy, rb_pts_z = [], []
        for ls in lanelet_map.lineStringLayer:
            try:
                if str(ls.attributes["type"]) != "road_border":
                    continue
            except Exception:
                continue
            for pt in ls:
                rb_pts_xy.append([pt.x, pt.y])
                rb_pts_z.append(pt.z)
        if not rb_pts_xy:
            return []

        rb_xy = np.array(rb_pts_xy)
        rb_z = np.array(rb_pts_z)
        tree = KDTree(rb_xy)

        MAX_AVG_DIST = 10.0
        shoulders = []
        for ls in lanelet_map.lineStringLayer:
            if ls.id not in boundary_usage or boundary_usage[ls.id] != 1:
                continue
            try:
                if str(ls.attributes["type"]) == "road_border":
                    continue
            except Exception:
                pass
            pts = list(ls)
            if len(pts) < 2:
                continue
            query = np.array([[p.x, p.y] for p in pts])
            dists, indices = tree.query(query)
            if np.mean(dists) > MAX_AVG_DIST or np.mean(dists) < 0.3:
                continue
            left_pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in pts]
            right_pts = [
                [round(float(rb_xy[idx][0]), 3), round(float(rb_xy[idx][1]), 3), round(float(rb_z[idx]), 3)]
                for idx in indices
            ]
            shoulders.append({"left": left_pts, "right": right_pts})
        return shoulders

    def _extract_road_markings(self, lanelet_map) -> list:
        MARKING_TYPES = {"line_thin", "line_thick", "stop_line"}
        markings = []
        for ls in lanelet_map.lineStringLayer:
            try:
                t = str(ls.attributes["type"])
            except Exception:
                continue
            if t not in MARKING_TYPES:
                continue
            pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ls]
            if len(pts) < 2:
                continue
            markings.append({"points": pts, "type": t})
        return markings


def main():
    rclpy.init()
    node = LaneletBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
