#!/usr/bin/env python3
"""Estimate Godot vehicle parameters from a rosbag2/MCAP log.

This fitter intentionally focuses on the subset that can be identified from:
  - /control/command/control_cmd
  - /localization/kinematic_state
  - /localization/acceleration
  - /vehicle/status/steering_status

It produces:
  - resistance parameter estimates
  - steering / accel / brake response delay and time constant estimates
  - a heuristic understeer estimate
  - a JSON report and an optional merged params JSON
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "ROS 2 Python packages are not available. Source /opt/ros/humble/setup.bash "
        "and your Autoware install/setup.bash before running this script."
    ) from exc


CONTROL_TOPIC = "/control/command/control_cmd"
ACTUATION_TOPIC = "/control/command/actuation_cmd"
ODOM_TOPIC = "/localization/kinematic_state"
ACCEL_TOPIC = "/localization/acceleration"
STEER_TOPIC = "/vehicle/status/steering_status"

G = 9.81


@dataclass
class SampleSeries:
    t: np.ndarray
    values: dict[str, np.ndarray]

    def require(self, *keys: str) -> None:
        if self.t.size == 0:
            raise ValueError("empty series")
        for key in keys:
            if key not in self.values:
                raise ValueError(f"missing key: {key}")


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_bag_series(bag_dir: Path) -> dict[str, SampleSeries]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in [CONTROL_TOPIC, ACTUATION_TOPIC, ODOM_TOPIC, ACCEL_TOPIC, STEER_TOPIC]
        if topic in topic_types
    }

    control_rows: list[tuple[float, float, float, float]] = []
    actuation_rows: list[tuple[float, float, float, float]] = []
    odom_rows: list[tuple[float, float, float, float, float, float]] = []
    accel_rows: list[tuple[float, float]] = []
    steer_rows: list[tuple[float, float]] = []

    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        if topic not in msg_types:
            continue

        msg = deserialize_message(raw, msg_types[topic])
        bag_time = float(bag_time_ns) * 1e-9

        if topic == CONTROL_TOPIC:
            t = stamp_to_sec(msg.stamp) if msg.stamp.sec != 0 else bag_time
            control_rows.append(
                (
                    t,
                    float(msg.longitudinal.acceleration),
                    float(msg.longitudinal.velocity),
                    float(msg.lateral.steering_tire_angle),
                )
            )
        elif topic == ACTUATION_TOPIC:
            t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec != 0 else bag_time
            actuation_rows.append(
                (
                    t,
                    float(msg.actuation.accel_cmd),
                    float(msg.actuation.brake_cmd),
                    float(msg.actuation.steer_cmd),
                )
            )
        elif topic == ODOM_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            pose = msg.pose.pose
            twist = msg.twist.twist
            yaw = quat_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            odom_rows.append(
                (
                    t,
                    float(pose.position.x),
                    float(pose.position.y),
                    float(yaw),
                    float(twist.linear.x),
                    float(twist.angular.z),
                )
            )
        elif topic == ACCEL_TOPIC:
            t = stamp_to_sec(msg.header.stamp)
            accel_rows.append((t, float(msg.accel.accel.linear.x)))
        elif topic == STEER_TOPIC:
            t = stamp_to_sec(msg.stamp)
            steer_rows.append((t, float(msg.steering_tire_angle)))

    if not control_rows or not odom_rows or not steer_rows:
        raise SystemExit("required topics are missing in the bag")

    control = np.asarray(control_rows, dtype=float)
    actuation = (
        np.asarray(actuation_rows, dtype=float)
        if actuation_rows
        else np.empty((0, 4), dtype=float)
    )
    odom = np.asarray(odom_rows, dtype=float)
    accel = np.asarray(accel_rows, dtype=float) if accel_rows else np.empty((0, 2), dtype=float)
    steer = np.asarray(steer_rows, dtype=float)

    def sort_rows(rows: np.ndarray) -> np.ndarray:
        if rows.size == 0:
            return rows
        return rows[np.argsort(rows[:, 0])]

    control = sort_rows(control)
    actuation = sort_rows(actuation)
    odom = sort_rows(odom)
    accel = sort_rows(accel)
    steer = sort_rows(steer)

    return {
        "control": SampleSeries(
            t=control[:, 0],
            values={
                "accel_cmd": control[:, 1],
                "vel_cmd": control[:, 2],
                "steer_cmd": control[:, 3],
            },
        ),
        "odom": SampleSeries(
            t=odom[:, 0],
            values={
                "x": odom[:, 1],
                "y": odom[:, 2],
                "yaw": odom[:, 3],
                "speed": odom[:, 4],
                "yaw_rate": odom[:, 5],
            },
        ),
        "actuation": SampleSeries(
            t=actuation[:, 0],
            values={
                "accel_cmd": actuation[:, 1] if actuation.size else np.empty(0, dtype=float),
                "brake_cmd": actuation[:, 2] if actuation.size else np.empty(0, dtype=float),
                "steer_cmd": actuation[:, 3] if actuation.size else np.empty(0, dtype=float),
            },
        ),
        "accel": SampleSeries(
            t=accel[:, 0],
            values={"ax": accel[:, 1]},
        ),
        "steer": SampleSeries(
            t=steer[:, 0],
            values={"steer_status": steer[:, 1]},
        ),
    }


def zoh_sample(times: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(times, query, side="right") - 1
    idx = np.clip(idx, 0, len(times) - 1)
    return values[idx]


def linear_sample(times: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.interp(query, times, values)


def smooth_signal(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / float(window)
    return np.convolve(padded, kernel, mode="valid")


def build_dataset(
    series: dict[str, SampleSeries],
    dt: float,
    start_offset: float,
    duration: float | None,
) -> dict[str, np.ndarray]:
    control = series["control"]
    actuation = series["actuation"]
    odom = series["odom"]
    accel = series["accel"]
    steer = series["steer"]

    start = max(control.t[0], odom.t[0], steer.t[0])
    end = min(control.t[-1], odom.t[-1], steer.t[-1])
    if actuation.t.size > 0:
        start = max(start, actuation.t[0])
        end = min(end, actuation.t[-1])
    if accel.t.size > 0:
        start = max(start, accel.t[0])
        end = min(end, accel.t[-1])

    start += start_offset
    if duration is not None:
        end = min(end, start + duration)
    if end <= start + dt:
        raise SystemExit("not enough overlapping data after applying offsets")

    t = np.arange(start, end, dt, dtype=float)
    yaw_unwrapped = np.unwrap(odom.values["yaw"])

    ax_obs = (
        linear_sample(accel.t, accel.values["ax"], t)
        if accel.t.size > 0
        else np.gradient(linear_sample(odom.t, odom.values["speed"], t), dt)
    )
    speed = linear_sample(odom.t, odom.values["speed"], t)
    speed_smooth = smooth_signal(speed, max(5, int(round(0.5 / dt))))
    ax_from_speed = np.gradient(speed_smooth, dt)

    data = {
        "t": t - t[0],
        "accel_cmd": zoh_sample(control.t, control.values["accel_cmd"], t),
        "vel_cmd": zoh_sample(control.t, control.values["vel_cmd"], t),
        "steer_cmd": zoh_sample(control.t, control.values["steer_cmd"], t),
        "actuation_accel_cmd": (
            zoh_sample(actuation.t, actuation.values["accel_cmd"], t)
            if actuation.t.size > 0 else np.zeros_like(t)
        ),
        "actuation_brake_cmd": (
            zoh_sample(actuation.t, actuation.values["brake_cmd"], t)
            if actuation.t.size > 0 else np.zeros_like(t)
        ),
        "actuation_steer_cmd": (
            zoh_sample(actuation.t, actuation.values["steer_cmd"], t)
            if actuation.t.size > 0 else np.zeros_like(t)
        ),
        "x": linear_sample(odom.t, odom.values["x"], t),
        "y": linear_sample(odom.t, odom.values["y"], t),
        "yaw": linear_sample(odom.t, yaw_unwrapped, t),
        "speed": speed,
        "yaw_rate": linear_sample(odom.t, odom.values["yaw_rate"], t),
        "ax": ax_obs,
        "ax_from_speed": ax_from_speed,
        "steer_status": linear_sample(steer.t, steer.values["steer_status"], t),
    }
    return data


def delay_then_lag(
    u: np.ndarray,
    dt: float,
    delay: float,
    tau: float,
    gain: float = 1.0,
    initial: float = 0.0,
) -> np.ndarray:
    delayed_t = np.arange(len(u), dtype=float) * dt - delay
    delayed_u = zoh_sample(
        np.arange(len(u), dtype=float) * dt,
        u,
        np.clip(delayed_t, 0.0, (len(u) - 1) * dt),
    )

    y = np.empty_like(u)
    y[0] = initial
    if tau <= 1e-6:
        y[:] = gain * delayed_u
        return y

    alpha = 1.0 - math.exp(-dt / tau)
    for i in range(1, len(u)):
        target = gain * delayed_u[i]
        y[i] = y[i - 1] + alpha * (target - y[i - 1])
    return y


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(a - b))))


def fit_fopdt(
    u: np.ndarray,
    y: np.ndarray,
    dt: float,
    active_mask: np.ndarray,
    delay_range: tuple[float, float],
    tau_range: tuple[float, float],
    gain_range: tuple[float, float],
    coarse_steps: int = 12,
    fine_steps: int = 8,
) -> dict[str, float]:
    if len(u) > 4000:
        stride = max(1, int(round(len(u) / 4000.0)))
        u = u[::stride]
        y = y[::stride]
        active_mask = active_mask[::stride]
        dt *= stride

    if active_mask.sum() < 20:
        return {"delay": 0.0, "tau": max(dt, 0.1), "gain": 1.0, "rmse": float("inf")}

    def evaluate(delay: float, tau: float, gain: float) -> float:
        pred = delay_then_lag(u, dt, delay, tau, gain=gain, initial=y[0])
        return rmse(pred[active_mask], y[active_mask])

    best = {"delay": 0.0, "tau": 0.1, "gain": 1.0, "rmse": float("inf")}
    for delay in np.linspace(delay_range[0], delay_range[1], coarse_steps):
        for tau in np.linspace(tau_range[0], tau_range[1], coarse_steps):
            for gain in np.linspace(gain_range[0], gain_range[1], coarse_steps):
                score = evaluate(float(delay), float(tau), float(gain))
                if score < best["rmse"]:
                    best = {"delay": float(delay), "tau": float(tau), "gain": float(gain), "rmse": score}

    delay_span = max((delay_range[1] - delay_range[0]) / coarse_steps, dt)
    tau_span = max((tau_range[1] - tau_range[0]) / coarse_steps, dt)
    gain_span = max((gain_range[1] - gain_range[0]) / coarse_steps, 0.01)

    for delay in np.linspace(max(delay_range[0], best["delay"] - delay_span),
                             min(delay_range[1], best["delay"] + delay_span), fine_steps):
        for tau in np.linspace(max(tau_range[0], best["tau"] - tau_span),
                               min(tau_range[1], best["tau"] + tau_span), fine_steps):
            for gain in np.linspace(max(gain_range[0], best["gain"] - gain_span),
                                    min(gain_range[1], best["gain"] + gain_span), fine_steps):
                score = evaluate(float(delay), float(tau), float(gain))
                if score < best["rmse"]:
                    best = {"delay": float(delay), "tau": float(tau), "gain": float(gain), "rmse": score}

    return best


def fit_resistance(data: dict[str, np.ndarray], params: dict[str, float]) -> dict[str, float]:
    speed = np.maximum(data["speed"], 0.0)
    coast_mask = (
        (np.abs(data["accel_cmd"]) < 0.05)
        & (np.abs(data["steer_cmd"]) < 0.03)
        & (speed > 3.0)
    )
    if coast_mask.sum() < 50:
        return {
            "rolling_resistance_coeff": float(params["rolling_resistance_coeff"]),
            "drag_coefficient": float(params["drag_coefficient"]),
            "engine_braking_force": float(params["engine_braking_force"]),
            "rmse": float("inf"),
            "samples": int(coast_mask.sum()),
        }

    mass = float(params["vehicle_weight"])
    frontal_area = float(params["frontal_area"])
    air_density = float(params["air_density"])
    v = speed[coast_mask]
    y = -data["ax_from_speed"][coast_mask]
    x = np.column_stack(
        [
            np.full_like(v, G),
            0.5 * air_density * frontal_area * v * v / mass,
            np.clip(v / 10.0, 0.1, 1.0) / mass,
        ]
    )
    beta, *_ = np.linalg.lstsq(x, y, rcond=None)
    beta = np.maximum(beta, 0.0)
    beta[0] = min(beta[0], 0.02)
    beta[1] = min(beta[1], 1.5)
    beta[2] = min(beta[2], 200.0)
    pred = x @ beta
    return {
        "rolling_resistance_coeff": float(beta[0]),
        "drag_coefficient": float(beta[1]),
        "engine_braking_force": float(beta[2]),
        "rmse": rmse(pred, y),
        "samples": int(coast_mask.sum()),
    }


def resistance_accel(speed: np.ndarray, params: dict[str, float]) -> np.ndarray:
    mass = float(params["vehicle_weight"])
    roll = float(params["rolling_resistance_coeff"]) * G
    drag = (
        0.5
        * float(params["air_density"])
        * float(params["drag_coefficient"])
        * float(params["frontal_area"])
        * np.square(speed)
        / mass
    )
    engine_brake = (
        float(params["engine_braking_force"])
        * np.clip(speed / 10.0, 0.1, 1.0)
        / mass
    )
    return roll + drag + engine_brake


def fit_understeer(data: dict[str, np.ndarray], params: dict[str, float]) -> dict[str, float]:
    speed = np.maximum(data["speed"], 0.1)
    delta_raw = data["steer_status"]
    delta_eff = np.arctan(float(params["wheel_base"]) * data["yaw_rate"] / speed)
    mask = (
        (speed > 5.0)
        & (np.abs(delta_raw) > 0.01)
        & (np.abs(delta_eff) > 0.003)
        & (np.sign(delta_raw) == np.sign(delta_eff))
    )
    if mask.sum() < 50:
        return {
            "understeer_gradient": float(params["understeer_gradient"]),
            "samples": int(mask.sum()),
        }

    kus_samples = ((delta_raw[mask] / delta_eff[mask]) - 1.0) * float(params["wheel_base"]) / np.square(speed[mask])
    kus_samples = kus_samples[np.isfinite(kus_samples)]
    kus_samples = kus_samples[(kus_samples > -0.1) & (kus_samples < 0.5)]
    if kus_samples.size == 0:
        return {
            "understeer_gradient": float(params["understeer_gradient"]),
            "samples": int(mask.sum()),
        }
    return {
        "understeer_gradient": float(max(np.median(kus_samples), 0.0)),
        "samples": int(kus_samples.size),
    }


def derive_actuation_accel_full_scale(params: dict[str, float], accel_gain: float) -> float:
    if accel_gain <= 1e-6:
        return float(params.get("actuation_accel_full_scale", 350.0))
    drive_gain = (
        2.0
        * float(params["max_engine_force"])
        * float(params["drivetrain_efficiency"])
        / float(params["vehicle_weight"])
    )
    if drive_gain <= 1e-6:
        return float(params.get("actuation_accel_full_scale", 350.0))
    return float(np.clip(drive_gain / accel_gain, 1.0, 10000.0))


def estimate_actuation_accel_full_scale(
    data: dict[str, np.ndarray],
    params: dict[str, float],
    resist_ax: np.ndarray,
) -> dict[str, float]:
    mask = (
        (data["actuation_accel_cmd"] > 10.0)
        & (data["speed"] > 1.0)
        & (np.abs(data["actuation_steer_cmd"]) < 0.05)
        & (data["ax_from_speed"] + resist_ax > 0.05)
    )
    if int(mask.sum()) < 50:
        return {
            "value": float(params.get("actuation_accel_full_scale", 350.0)),
            "samples": int(mask.sum()),
        }

    numerator = (
        2.0
        * float(params["max_engine_force"])
        * float(params["drivetrain_efficiency"])
        * data["actuation_accel_cmd"][mask]
    )
    denominator = float(params["vehicle_weight"]) * (data["ax_from_speed"][mask] + resist_ax[mask])
    scale = numerator / denominator
    scale = scale[np.isfinite(scale)]
    scale = scale[(scale > 1.0) & (scale < 20000.0)]
    if scale.size == 0:
        return {
            "value": float(params.get("actuation_accel_full_scale", 350.0)),
            "samples": int(mask.sum()),
        }
    return {
        "value": float(np.median(scale)),
        "samples": int(scale.size),
        "p25": float(np.percentile(scale, 25)),
        "p75": float(np.percentile(scale, 75)),
    }


def round_param(value: float) -> float:
    return float(np.round(value, 6))


def merge_params(base: dict[str, float], updates: dict[str, float]) -> dict[str, float]:
    merged = dict(base)
    merged.update({k: round_param(v) for k, v in updates.items()})
    return merged


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--params-json", type=Path, default=Path("vehicle_params.json"))
    parser.add_argument("--output-report", type=Path)
    parser.add_argument("--write-params", type=Path)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float)
    args = parser.parse_args()

    if not args.bag_dir.exists():
        raise SystemExit(f"bag dir not found: {args.bag_dir}")
    if not args.params_json.exists():
        raise SystemExit(f"params JSON not found: {args.params_json}")

    with args.params_json.open() as fp:
        params = json.load(fp)

    series = read_bag_series(args.bag_dir)
    data = build_dataset(series, args.dt, args.start_offset, args.duration)

    resistance_fit = fit_resistance(data, params)
    fitted_for_dynamics = merge_params(
        params,
        {
            "rolling_resistance_coeff": resistance_fit["rolling_resistance_coeff"],
            "drag_coefficient": resistance_fit["drag_coefficient"],
            "engine_braking_force": resistance_fit["engine_braking_force"],
        },
    )
    resist_ax = resistance_accel(np.maximum(data["speed"], 0.0), fitted_for_dynamics)

    actuation_available = series["actuation"].t.size > 0
    steer_input = data["actuation_steer_cmd"] if actuation_available else data["steer_cmd"]
    accel_input = data["actuation_accel_cmd"] if actuation_available else np.maximum(data["accel_cmd"], 0.0)
    brake_input = data["actuation_brake_cmd"] if actuation_available else np.maximum(-data["accel_cmd"], 0.0)

    steer_mask = (np.abs(steer_input) > 0.01) | (np.abs(data["steer_status"]) > 0.01)
    steer_fit = fit_fopdt(
        steer_input,
        data["steer_status"],
        args.dt,
        active_mask=steer_mask,
        delay_range=(0.0, 0.6),
        tau_range=(0.02, 0.5),
        gain_range=(0.8, 1.2),
    )

    accel_target = np.maximum(data["ax_from_speed"] + resist_ax, 0.0)
    accel_mask = (accel_input > (1.0 if actuation_available else 0.5)) & (data["speed"] > 1.0) & (np.abs(steer_input) < 0.05)
    accel_fit = fit_fopdt(
        accel_input,
        accel_target,
        args.dt,
        active_mask=accel_mask,
        delay_range=(0.0, 1.0),
        tau_range=(0.02, 0.8),
        gain_range=(0.001, 1.5) if actuation_available else (0.5, 1.5),
    )

    brake_target = np.maximum(-data["ax_from_speed"], 0.0)
    brake_mask = (brake_input > (0.1 if actuation_available else 0.5)) & (data["speed"] > 1.0) & (np.abs(steer_input) < 0.05)
    brake_fit = fit_fopdt(
        brake_input,
        brake_target,
        args.dt,
        active_mask=brake_mask,
        delay_range=(0.0, 0.5),
        tau_range=(0.02, 0.5),
        gain_range=(0.5, 1.5),
    )

    understeer_fit = fit_understeer(data, params)
    actuation_scale_fit = (
        estimate_actuation_accel_full_scale(data, params, resist_ax)
        if actuation_available else None
    )

    notes = [
        "max_engine_force, max_brake_force, wheel_friction_slip, steer_speed_threshold, and steer_high_speed_ratio are not reliably identifiable from this log alone.",
        "full_brake_decel and actuation_accel_full_scale are heuristic back-solves from gain fits; verify them by replaying the updated params.",
    ]

    suggested = {
        "accel_response_delay": accel_fit["delay"],
        "brake_response_delay": brake_fit["delay"],
        "steering_response_delay": steer_fit["delay"],
        "accel_time_constant": accel_fit["tau"],
        "brake_time_constant": brake_fit["tau"],
        "steering_time_constant": steer_fit["tau"],
        "understeer_gradient": understeer_fit["understeer_gradient"],
    }

    if 0.0 < resistance_fit["rolling_resistance_coeff"] < 0.02:
        suggested["rolling_resistance_coeff"] = resistance_fit["rolling_resistance_coeff"]
    else:
        notes.append("rolling_resistance_coeff hit the fit bound; keep the current value unless a coast-only log is available.")

    if 0.0 < resistance_fit["drag_coefficient"] < 1.5:
        suggested["drag_coefficient"] = resistance_fit["drag_coefficient"]
    else:
        notes.append("drag_coefficient hit the fit bound; keep the current value unless a longer straight coast segment is available.")

    if 0.0 < resistance_fit["engine_braking_force"] < 200.0:
        suggested["engine_braking_force"] = resistance_fit["engine_braking_force"]
    else:
        notes.append("engine_braking_force could not be separated cleanly from rolling resistance in this bag.")

    if actuation_available and actuation_scale_fit is not None:
        suggested["actuation_accel_full_scale"] = actuation_scale_fit["value"]
    elif np.isfinite(accel_fit["gain"]) and accel_fit["gain"] > 0.05:
        suggested["drivetrain_efficiency"] = float(np.clip(1.0 / accel_fit["gain"], 0.5, 1.0))
    if np.isfinite(brake_fit["gain"]) and brake_fit["gain"] > 0.05:
        suggested["full_brake_decel"] = float(
            np.clip(float(params["full_brake_decel"]) * brake_fit["gain"], 0.5, 4.0)
        )

    if brake_fit["tau"] >= 0.49 or brake_fit["delay"] >= 0.49:
        notes.append("brake response fit landed on its search bound; treat brake_response_* as provisional.")

    report = {
        "bag_dir": str(args.bag_dir),
        "params_json": str(args.params_json),
        "fit_window_sec": round_param(float(data["t"][-1] - data["t"][0])),
        "dt_sec": round_param(args.dt),
        "sample_counts": {
            "control": int(series["control"].t.size),
            "actuation": int(series["actuation"].t.size),
            "odom": int(series["odom"].t.size),
            "accel": int(series["accel"].t.size),
            "steering": int(series["steer"].t.size),
            "resampled": int(data["t"].size),
        },
        "diagnostics": {
            "resistance_fit": {
                "samples": resistance_fit["samples"],
                "rmse_mps2": round_param(resistance_fit["rmse"]),
            },
            "steering_fit": {
                "samples": int(steer_mask.sum()),
                "gain": round_param(steer_fit["gain"]),
                "rmse_rad": round_param(steer_fit["rmse"]),
            },
            "accel_fit": {
                "samples": int(accel_mask.sum()),
                "gain": round_param(accel_fit["gain"]),
                "rmse_mps2": round_param(accel_fit["rmse"]),
                "input_topic": ACTUATION_TOPIC if actuation_available else CONTROL_TOPIC,
            },
            "brake_fit": {
                "samples": int(brake_mask.sum()),
                "gain": round_param(brake_fit["gain"]),
                "rmse_mps2": round_param(brake_fit["rmse"]),
                "input_topic": ACTUATION_TOPIC if actuation_available else CONTROL_TOPIC,
            },
            "understeer_fit": {
                "samples": understeer_fit["samples"],
            },
            "actuation_scale_fit": (
                {
                    "samples": actuation_scale_fit["samples"],
                    "p25": round_param(actuation_scale_fit.get("p25", actuation_scale_fit["value"])),
                    "p75": round_param(actuation_scale_fit.get("p75", actuation_scale_fit["value"])),
                    "value": round_param(actuation_scale_fit["value"]),
                }
                if actuation_scale_fit is not None else None
            ),
        },
        "suggested_params": {k: round_param(v) for k, v in suggested.items()},
        "notes": notes,
    }

    print(json.dumps(report, indent=2, sort_keys=True))

    if args.output_report:
        args.output_report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    if args.write_params:
        merged = merge_params(params, report["suggested_params"])
        args.write_params.write_text(json.dumps(merged, indent=2, sort_keys=False) + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
