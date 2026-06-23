# Copyright 2026 Tatsukiyano
import json
import os
import sys
import glob
from datetime import datetime, timezone

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import minimize
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String

WHEEL_RADIUS = 0.05
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]
VELOCITY_UNIT = "m/s"
COHERENCE_MIN = 0.6


def stamp_to_sec(msg, bag_time_ns):
    t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    if t_msg < 1e-3:
        t_msg = bag_time_ns / 1e9
    return t_msg


def extract_bag_data(bag_path):
    storage_id = "sqlite3" if glob.glob(os.path.join(bag_path, "*.db3")) else "mcap"
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open bag: {e}")
        return None

    results = {
        "cmd": [],
        "cmd_stab": [],
        "wheel_odom": [],
        "ekf": [],
        "gt": [],
        "joint_vx": [],
        "effort": [],
        "phase": [],
        "imu_accel": [],
        "imu_gyro": [],
    }

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == "/cmd_vel_ext":
            msg = deserialize_message(data, TwistStamped)
            results["cmd"].append((stamp_to_sec(msg, t), msg.twist.linear.x))
        elif topic == "/mecanum_drive_controller/reference":
            msg = deserialize_message(data, TwistStamped)
            results["cmd_stab"].append((stamp_to_sec(msg, t), msg.twist.linear.x))
        elif topic == "/mecanum_drive_controller/odometry":
            msg = deserialize_message(data, Odometry)
            vel = msg.twist.twist.linear.x
            if abs(vel) < 20.0:
                results["wheel_odom"].append((stamp_to_sec(msg, t), vel))
        elif topic == "/odometry/filtered":
            msg = deserialize_message(data, Odometry)
            vel = msg.twist.twist.linear.x
            if abs(vel) < 20.0:
                results["ekf"].append((stamp_to_sec(msg, t), vel))
        elif topic == "/odom/ground_truth":
            msg = deserialize_message(data, Odometry)
            results["gt"].append((stamp_to_sec(msg, t), msg.twist.twist.linear.x))
        elif topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            vx = mecanum_vx_from_joint_state(msg)
            if vx is not None:
                results["joint_vx"].append((stamp_to_sec(msg, t), vx))
            if msg.effort:
                results["effort"].append((stamp_to_sec(msg, t), np.mean(np.abs(msg.effort))))
        elif topic == "/control_analysis/phase":
            msg = deserialize_message(data, String)
            results["phase"].append((t / 1e9, msg.data))
        elif topic == "/imu":
            msg = deserialize_message(data, Imu)
            results["imu_accel"].append((stamp_to_sec(msg, t), msg.linear_acceleration.x))
            results["imu_gyro"].append((stamp_to_sec(msg, t), msg.angular_velocity.z))

    return {k: np.array(v) for k, v in results.items() if len(v) > 0}


def mecanum_vx_from_joint_state(msg):
    if not msg.velocity or not msg.name:
        return None
    name_to_vel = dict(zip(msg.name, msg.velocity))
    speeds = [name_to_vel.get(joint) for joint in WHEEL_JOINTS]
    if any(v is None for v in speeds):
        if len(msg.velocity) >= 4:
            speeds = msg.velocity[:4]
        else:
            return None
    return WHEEL_RADIUS / 4.0 * float(np.sum(speeds))


def normalize_time(data):
    available = [float(data[key][0, 0]) for key in data if len(data[key]) > 0]
    if not available:
        return data
    t0 = min(available)
    for key in data:
        if key == "phase":
            for i in range(len(data[key])):
                data[key][i, 0] = str(float(data[key][i, 0]) - t0)
        else:
            data[key][:, 0] -= t0
    return data


def estimate_sample_rate(t):
    if len(t) < 2:
        return 100.0
    dt = np.diff(t)
    dt = dt[(dt > 1e-4) & (dt < 0.5)]
    if len(dt) == 0:
        return 100.0
    return float(np.clip(1.0 / np.median(dt), 20.0, 500.0))


def select_analysis_signals(data):
    warnings = []

    if "cmd_stab" in data:
        u_key, u_label = "cmd_stab", "Controller Reference"
    elif "cmd" in data:
        u_key, u_label = "cmd", "External Command"
        warnings.append("Controller reference missing; using /cmd_vel_ext as input.")
    else:
        return None, None, None, None, warnings, None, None, None

    response_priority = [
        ("gt", "Ground Truth"),
        ("wheel_odom", "Wheel Odometry"),
        ("ekf", "EKF Estimate"),
        ("joint_vx", "Joint Kinematics"),
    ]
    y_key, y_label = None, None
    for key, label in response_priority:
        if key in data:
            y_key, y_label = key, label
            break

    if y_key is None:
        warnings.append("No response signal found in bag.")
        return None, None, None, None, warnings, u_label, None, None

    if y_key == "ekf":
        warnings.append(
            "Using EKF output as plant response. Record /mecanum_drive_controller/odometry "
            "for accurate drive plant identification."
        )

    if "gt" not in data:
        warnings.append("Ground truth not recorded. Bridge /model/rox2026/odometry in simulation.")

    if "wheel_odom" in data and "ekf" in data:
        t_common = np.linspace(
            max(data["wheel_odom"][0, 0], data["ekf"][0, 0]),
            min(data["wheel_odom"][-1, 0], data["ekf"][-1, 0]),
            200,
        )
        wheel_interp = np.interp(t_common, data["wheel_odom"][:, 0], data["wheel_odom"][:, 1])
        ekf_interp = np.interp(t_common, data["ekf"][:, 0], data["ekf"][:, 1])
        denom = max(np.std(wheel_interp), 1e-3)
        mismatch = float(np.mean(np.abs(wheel_interp - ekf_interp)) / denom)
        if mismatch > 0.25:
            warnings.append(
                f"Wheel odometry and EKF disagree (normalized error {mismatch:.2f}). "
                "Prefer wheel odometry for plant ID."
            )

    return (
        data[u_key][:, 0],
        data[u_key][:, 1],
        data[y_key][:, 0],
        data[y_key][:, 1],
        warnings,
        u_label,
        y_label,
        y_key,
    )


def detect_segments(data, t_cmd, u_cmd):
    prbs_t0, prbs_t1 = None, None
    if "phase" in data:
        phase = data["phase"]
        chirp_start = phase[phase[:, 1] == "CHIRP_START"]
        chirp_end = phase[phase[:, 1] == "CHIRP_END"]
        step_start = phase[phase[:, 1] == "STEP_START"]
        prbs_start = phase[phase[:, 1] == "PRBS_START"]
        prbs_end = phase[phase[:, 1] == "PRBS_END"]

        step_end = phase[phase[:, 1] == "STEP_END"]

        if len(step_start) > 0:
            t_step = float(step_start[0, 0])
            step_t1 = float(step_end[0, 0]) if len(step_end) > 0 else t_cmd[-1]
            chirp_t0 = float(chirp_start[0, 0]) if len(chirp_start) else 1.0
            chirp_t1 = (
                float(chirp_end[0, 0]) if len(chirp_end) else max(t_step - 1.0, chirp_t0 + 1.0)
            )
            if len(prbs_start) > 0:
                prbs_t0 = float(prbs_start[0, 0])
                prbs_t1 = float(prbs_end[0, 0]) if len(prbs_end) else t_cmd[-1]
            return t_step, step_t1, chirp_t0, chirp_t1, prbs_t0, prbs_t1, "phase_marker"

    step_indices = np.where(u_cmd >= 1.9)[0]
    if len(step_indices) == 0:
        diffs = np.abs(np.diff(u_cmd))
        step_indices = np.where(diffs > 0.5)[0]

    t_step = float(t_cmd[step_indices[0]] if len(step_indices) > 0 else t_cmd[-1] - 5.0)
    step_t1 = t_cmd[-1]
    chirp_t0 = 1.0
    chirp_t1 = max(t_step - 1.0, chirp_t0 + 1.0)
    return t_step, step_t1, chirp_t0, chirp_t1, prbs_t0, prbs_t1, "heuristic"


def analyze_step_response(t_cmd, u_cmd, t_resp, y_resp, t_step, step_t1):
    t_end = step_t1
    duration = t_end - t_step

    mask_resp = (t_resp >= t_step) & (t_resp <= t_end)
    t_step_data = t_resp[mask_resp]
    y_step_data = y_resp[mask_resp]

    mask_cmd = (t_cmd >= t_step) & (t_cmd <= t_end)
    u_cmd_data = u_cmd[mask_cmd]

    if len(t_step_data) < 10 or len(u_cmd_data) < 2:
        return None

    u_baseline = np.median(u_cmd[t_cmd < t_step]) if any(t_cmd < t_step) else 0.0
    u_target = np.median(u_cmd_data[int(len(u_cmd_data) * 0.5) :]) if len(u_cmd_data) > 0 else 0.0
    u_step = u_target - u_baseline

    t_norm = t_step_data - t_step
    y_norm = y_step_data - u_baseline

    ss_mask = t_norm > (0.8 * duration)
    y_ss = np.mean(y_norm[ss_mask]) if any(ss_mask) else y_norm[-1]

    K_p = y_ss / u_step if abs(u_step) > 1e-5 else 0.0

    if u_step > 0:
        y_peak = np.max(y_norm)
        idx_peak = np.argmax(y_norm)
        overshoot = max(0.0, (y_peak - y_ss) / abs(y_ss)) * 100.0 if abs(y_ss) > 1e-5 else 0.0
    else:
        y_peak = np.min(y_norm)
        idx_peak = np.argmin(y_norm)
        overshoot = max(0.0, (y_ss - y_peak) / abs(y_ss)) * 100.0 if abs(y_ss) > 1e-5 else 0.0

    t_peak = t_norm[idx_peak]

    val_10 = 0.1 * y_ss
    val_90 = 0.9 * y_ss
    t_10, t_90 = None, None
    for t_val, y_val in zip(t_norm, y_norm):
        if u_step > 0:
            if t_10 is None and y_val >= val_10:
                t_10 = t_val
            if t_90 is None and y_val >= val_90:
                t_90 = t_val
        else:
            if t_10 is None and y_val <= val_10:
                t_10 = t_val
            if t_90 is None and y_val <= val_90:
                t_90 = t_val

    rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

    err = np.abs(y_norm - y_ss)
    thresh_2 = 0.02 * abs(y_ss)
    idx_out_2 = np.where(err > thresh_2)[0]
    settling_time_2 = t_norm[idx_out_2[-1]] if len(idx_out_2) > 0 else 0.0

    def fopdt_response(t, K, tau, L):
        resp = np.zeros_like(t)
        active = t >= L
        resp[active] = K * u_step * (1.0 - np.exp(-(t[active] - L) / tau))
        return resp

    def loss_func(params):
        K, tau, L = params
        y_fit = fopdt_response(t_norm, K, tau, L)
        return np.sum((y_norm - y_fit) ** 2)

    val_63 = 0.632 * y_ss
    idx_63 = np.where(y_norm >= val_63 if u_step > 0 else y_norm <= val_63)[0]
    t_63 = t_norm[idx_63[0]] if len(idx_63) > 0 else 0.5

    res = minimize(
        loss_func,
        [K_p, max(0.01, t_63), 0.05],
        bounds=[(0.01, 10.0), (0.01, 5.0), (0.0, 2.0)],
        method="L-BFGS-B",
    )
    K_fit, tau_fit, L_fit = res.x

    y_fit = fopdt_response(t_norm, K_fit, tau_fit, L_fit)
    ss_res = np.sum((y_norm - y_fit) ** 2)
    ss_tot = np.sum((y_norm - np.mean(y_norm)) ** 2)
    r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    return {
        "t_norm": t_norm,
        "y_norm": y_norm + u_baseline,
        "y_fit": y_fit + u_baseline,
        "K_p": K_p,
        "u_step": u_step,
        "y_ss": y_ss + u_baseline,
        "overshoot": overshoot,
        "t_peak": t_peak,
        "rise_time": rise_time,
        "settling_time_2": settling_time_2,
        "ss_error": u_target - (y_ss + u_baseline),
        "fopdt_K": K_fit,
        "fopdt_tau": tau_fit,
        "fopdt_L": L_fit,
        "fopdt_r2": r_squared,
    }


def analyze_frequency_response(
    t_cmd, u_cmd, t_resp, y_resp, chirp_t0, chirp_t1, prbs_t0=None, prbs_t1=None
):
    duration = chirp_t1 - chirp_t0
    if duration <= 0.0 or duration > 100.0:
        print(f"Error: Invalid chirp duration {duration:.2f}s. Check phase markers or time sync.")
        return None

    fs = estimate_sample_rate(t_resp)
    dt = 1.0 / fs

    def get_uniform(t0, t1):
        t_uni = np.arange(t0, t1, dt)
        u_uni = np.interp(t_uni, t_cmd, u_cmd)
        y_uni = np.interp(t_uni, t_resp, y_resp)
        return signal.detrend(u_uni), signal.detrend(y_uni)

    u_chirp, y_chirp = get_uniform(chirp_t0, chirp_t1)
    if prbs_t0 is not None and prbs_t1 is not None and (prbs_t1 - prbs_t0) > 1.0:
        u_prbs, y_prbs = get_uniform(prbs_t0, prbs_t1)
        u_detrend = np.concatenate([u_chirp, u_prbs])
        y_detrend = np.concatenate([y_chirp, y_prbs])
    else:
        u_detrend = u_chirp
        y_detrend = y_chirp

    nperseg = int(np.clip(len(u_detrend) // 4, 128, 512))
    f, Pxx = signal.welch(u_detrend, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    f, Pyy = signal.welch(y_detrend, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    f, Pxy = signal.csd(u_detrend, y_detrend, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)

    H = Pxy / (Pxx + 1e-12)
    coherence = np.abs(Pxy) ** 2 / (Pxx * Pyy + 1e-12)

    valid = (f >= 0.1) & (f <= 20.0) & (coherence >= COHERENCE_MIN)
    if not np.any(valid):
        valid = (f >= 0.1) & (f <= 20.0)

    f_valid = f[valid]
    H_valid = H[valid]
    coh_valid = coherence[valid]
    mag_db = 20 * np.log10(np.abs(H_valid) + 1e-12)
    phase_deg = np.rad2deg(np.unwrap(np.angle(H_valid)))

    mag_db_all = 20 * np.log10(np.abs(H) + 1e-12)
    phase_deg_all = np.rad2deg(np.unwrap(np.angle(H)))

    f_cg, phase_margin = None, None
    f_cp, gain_margin = None, None
    if len(f_valid) > 2:
        try:
            cs_mag = CubicSpline(f_valid, mag_db)
            cs_phase = CubicSpline(f_valid, phase_deg)
            f_dense = np.logspace(np.log10(f_valid[0]), np.log10(f_valid[-1]), 1000)
            mag_dense = cs_mag(f_dense)
            phase_dense = cs_phase(f_dense)

            crossings_cg = np.where(np.diff(np.sign(mag_dense)))[0]
            if len(crossings_cg) > 0:
                idx = crossings_cg[0]
                f_cg = float(f_dense[idx])
                phase_margin = float(phase_dense[idx] + 180.0)

            crossings_cp = np.where(np.diff(np.sign(phase_dense + 180.0)))[0]
            if len(crossings_cp) > 0:
                idx = crossings_cp[0]
                f_cp = float(f_dense[idx])
                gain_margin = float(-mag_dense[idx])
        except Exception:
            pass

    return {
        "freq": f_valid,
        "H": H_valid,
        "mag_db": mag_db,
        "phase_deg": phase_deg,
        "coherence": coh_valid,
        "sample_rate_hz": fs,
        "f_cg": f_cg,
        "phase_margin": phase_margin,
        "f_cp": f_cp,
        "gain_margin": gain_margin,
        "f_all": f,
        "H_all": H,
        "Pxx": Pxx,
        "Pyy": Pyy,
        "coherence_all": coherence,
        "mag_db_all": mag_db_all,
        "phase_deg_all": phase_deg_all,
    }


def print_ascii_report(
    time_metrics, freq_metrics, warnings, input_label, output_label, segment_method
):
    print("\n" + "=" * 60)
    print("           CONTROL SYSTEM IDENTIFICATION REPORT (PRO)")
    print("=" * 60)
    print(f"  Input Signal           : {input_label}")
    print(f"  Output Signal          : {output_label}")
    print(f"  Segment Detection      : {segment_method}")

    if warnings:
        print("\n[Data Quality Warnings]")
        for warning in warnings:
            print(f"  ! {warning}")

    if time_metrics:
        print("\n[Time Domain: Step Response Metrics]")
        print(f"  Step Command Jump      : {time_metrics['u_step']:.2f} {VELOCITY_UNIT}")
        print(f"  Steady-State Value     : {time_metrics['y_norm'][-1]:.2f} {VELOCITY_UNIT}")
        print(f"  Steady-State Gain (Kp) : {time_metrics['K_p']:.4f}")
        print(f"  Steady-State Error     : {time_metrics['ss_error']:.4f} {VELOCITY_UNIT}")
        print(f"  Overshoot (Mp)         : {time_metrics['overshoot']:.2f} %")
        print(f"  Peak Time (tp)         : {time_metrics['t_peak']:.3f} s")
        print(
            f"  Rise Time (tr, 10-90%) : {time_metrics['rise_time']:.3f} s"
            if time_metrics["rise_time"]
            else "  Rise Time (tr, 10-90%) : N/A"
        )
        print(f"  Settling Time (ts, 2%) : {time_metrics['settling_time_2']:.3f} s")
        print("\n[Estimated System Transfer Function (FOPDT)]")
        print("  Model                  : G(s) = K * exp(-L*s) / (tau*s + 1)")
        print(f"  Process Gain (K)       : {time_metrics['fopdt_K']:.4f}")
        print(f"  Time Constant (tau)    : {time_metrics['fopdt_tau']:.4f} s")
        print(f"  Dead Time / Delay (L)  : {time_metrics['fopdt_L']:.4f} s")
        print(f"  Model Accuracy (R^2)   : {time_metrics['fopdt_r2'] * 100:.2f} %")

    if freq_metrics:
        print("\n[Frequency Domain: Dynamic Margins]")
        print(f"  Resample Rate          : {freq_metrics['sample_rate_hz']:.1f} Hz")
        print(f"  Coherence Threshold    : {COHERENCE_MIN:.2f}")
        print(
            f"  Gain Crossover (f_cg)  : {freq_metrics['f_cg']:.3f} Hz"
            if freq_metrics["f_cg"]
            else "  Gain Crossover (f_cg)  : N/A"
        )
        print(
            f"  Phase Margin (PM)      : {freq_metrics['phase_margin']:.2f} deg"
            if freq_metrics["phase_margin"] is not None
            else "  Phase Margin (PM)      : N/A"
        )
        print(
            f"  Phase Crossover (f_cp) : {freq_metrics['f_cp']:.3f} Hz"
            if freq_metrics["f_cp"]
            else "  Phase Crossover (f_cp) : N/A"
        )
        print(
            f"  Gain Margin (GM)       : {freq_metrics['gain_margin']:.2f} dB"
            if freq_metrics["gain_margin"] is not None
            else "  Gain Margin (GM)       : inf (Stable)"
        )
    print("=" * 60 + "\n")


def export_json_report(
    bag_path,
    report_name,
    time_metrics,
    freq_metrics,
    warnings,
    input_label,
    output_label,
    segment_method,
    output_dir,
):
    slim_time = None
    if time_metrics:
        slim_time = {
            k: v for k, v in time_metrics.items() if k not in ("t_norm", "y_norm", "y_fit")
        }

    slim_freq = None
    if freq_metrics:
        slim_freq = {
            k: v
            for k, v in freq_metrics.items()
            if k
            not in (
                "freq",
                "H",
                "mag_db",
                "phase_deg",
                "coherence",
                "f_all",
                "Pxx",
                "Pyy",
                "coherence_all",
                "H_all",
                "mag_db_all",
                "phase_deg_all",
            )
        }

    payload = {
        "report_name": report_name,
        "bag": os.path.basename(bag_path),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "input_signal": input_label,
        "output_signal": output_label,
        "segment_detection": segment_method,
        "warnings": warnings,
        "time_domain": slim_time,
        "frequency_domain": slim_freq,
    }
    json_path = os.path.join(output_dir, f"{report_name}.json")
    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=_json_default)
    return json_path


def _json_default(value):
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"Object of type {type(value)} is not JSON serializable")


def run_pro_analysis(bag_path, report_name="control_report", output_dir="."):
    print(f"\n=== [PRO] Starting High-End Control Analysis for: {bag_path} ===")
    data = extract_bag_data(bag_path)
    if not data:
        print("Error: Could not read bag file.")
        return 1

    data = normalize_time(data)
    selection = select_analysis_signals(data)
    if selection[0] is None:
        print("Error: Insufficient command or response data in bag file.")
        return 1

    t_cmd, u_cmd, t_resp, y_resp, warnings, input_label, output_label, _ = selection
    seg_t = data["cmd"][:, 0] if "cmd" in data else t_cmd
    seg_u = data["cmd"][:, 1] if "cmd" in data else u_cmd
    t_step, step_t1, chirp_t0, chirp_t1, prbs_t0, prbs_t1, segment_method = detect_segments(
        data, seg_t, seg_u
    )

    time_metrics = analyze_step_response(t_cmd, u_cmd, t_resp, y_resp, t_step, step_t1)
    freq_metrics = None
    if chirp_t1 - chirp_t0 > 1.0:
        freq_metrics = analyze_frequency_response(
            t_cmd, u_cmd, t_resp, y_resp, chirp_t0, chirp_t1, prbs_t0, prbs_t1
        )

    print_ascii_report(
        time_metrics, freq_metrics, warnings, input_label, output_label, segment_method
    )

    os.makedirs(output_dir, exist_ok=True)
    png_path = os.path.join(output_dir, f"{report_name}.png")
    json_path = export_json_report(
        bag_path,
        report_name,
        time_metrics,
        freq_metrics,
        warnings,
        input_label,
        output_label,
        segment_method,
        output_dir,
    )

    _render_dashboard(
        bag_path,
        data,
        t_step,
        time_metrics,
        freq_metrics,
        input_label,
        output_label,
        png_path,
    )

    print(f">>> SUCCESS: {png_path} generated <<<")
    print(f">>> SUCCESS: {json_path} generated <<<")
    return 0


def _render_dashboard(
    bag_path,
    data,
    t_step,
    time_metrics,
    freq_metrics,
    input_label,
    output_label,
    png_path,
):
    fig, axs = plt.subplots(5, 2, figsize=(16, 20))
    plt.suptitle(
        f"Control Engineering Performance Dashboard\nBag: {os.path.basename(bag_path)}",
        fontsize=16,
        fontweight="bold",
    )

    ax1 = axs[0, 0]
    if "cmd" in data:
        ax1.step(
            data["cmd"][:, 0],
            data["cmd"][:, 1],
            "r--",
            label="External Cmd",
            where="post",
            alpha=0.5,
        )
    if "cmd_stab" in data:
        ax1.step(
            data["cmd_stab"][:, 0],
            data["cmd_stab"][:, 1],
            "m--",
            label=input_label,
            where="post",
            alpha=0.7,
        )
    response_layers = [
        ("wheel_odom", "Wheel Odom", "c"),
        ("joint_vx", "Joint FK", "orange"),
        ("ekf", "EKF", "b"),
        ("gt", "Ground Truth", "g"),
    ]
    for key, label, color in response_layers:
        if key in data:
            ax1.plot(data[key][:, 0], data[key][:, 1], color=color, label=label, lw=1.8, alpha=0.85)
    ax1.axvline(t_step, color="k", linestyle=":", alpha=0.4)
    ax1.set_title("Step Response (Time Domain)")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel(f"Velocity [{VELOCITY_UNIT}]")
    ax1.set_xlim(
        max(0.0, t_step - 1.0), data["cmd_stab"][-1, 0] if "cmd_stab" in data else t_step + 6.0
    )
    ax1.grid(True, linestyle=":")
    ax1.legend(loc="lower right", fontsize=8)

    ax2 = axs[0, 1]
    if "cmd_stab" in data and time_metrics:
        primary = next(
            (data[key] for key in ["wheel_odom", "joint_vx", "ekf", "gt"] if key in data),
            None,
        )
        if primary is not None:
            ref_interp = np.interp(primary[:, 0], data["cmd_stab"][:, 0], data["cmd_stab"][:, 1])
            error = ref_interp - primary[:, 1]
            ax2.plot(primary[:, 0], error, "r-", label=f"Tracking Error ({output_label})")
            ax2.set_ylabel(f"Error [{VELOCITY_UNIT}]", color="r")
            ax2.tick_params(axis="y", labelcolor="r")
            ax2.set_xlim(max(0.0, t_step - 1.0), primary[-1, 0])
            ax2.grid(True, linestyle=":")
            if "effort" in data:
                ax2_right = ax2.twinx()
                ax2_right.plot(
                    data["effort"][:, 0],
                    data["effort"][:, 1],
                    "g-",
                    alpha=0.6,
                    label="Mean Torque",
                )
                ax2_right.set_ylabel("Actuator Effort [Nm]", color="g")
                ax2_right.tick_params(axis="y", labelcolor="g")
            ax2.set_title("Control Accuracy and Effort")
            ax2.set_xlabel("Time [s]")

    ax3 = axs[1, 0]
    if freq_metrics and "mag_db_all" in freq_metrics:
        f_all = freq_metrics["f_all"]
        mag_all = freq_metrics["mag_db_all"]
        coh_all = freq_metrics["coherence_all"]
        try:
            from scipy.signal import savgol_filter

            win = min(21, len(mag_all) | 1)
            mag_smooth = savgol_filter(mag_all, window_length=win, polyorder=3)
            phase_smooth = savgol_filter(
                freq_metrics["phase_deg_all"], window_length=win, polyorder=3
            )
        except Exception:
            mag_smooth = mag_all
            phase_smooth = freq_metrics["phase_deg_all"]

        ax3.semilogx(f_all, mag_smooth, "b-", lw=2, label="Magnitude")
        ax3.fill_between(
            f_all,
            -100,
            100,
            where=(coh_all < COHERENCE_MIN),
            color="gray",
            alpha=0.3,
            label="Noise Region",
        )
        ax3.set_title("Bode Magnitude Plot")
        ax3.set_xlabel("Frequency [Hz]")
        ax3.set_ylabel("Magnitude [dB]")
        ax3.set_ylim(-40, 20)
        ax3.grid(True, which="both", linestyle=":")
        if freq_metrics["f_cg"]:
            ax3.axvline(
                freq_metrics["f_cg"],
                color="r",
                linestyle="--",
                label=f"Crossover: {freq_metrics['f_cg']:.2f}Hz",
            )
        ax3.legend(loc="lower left", fontsize=8)

    ax4 = axs[1, 1]
    if freq_metrics and "phase_deg_all" in freq_metrics:
        ax4.semilogx(f_all, phase_smooth, "b-", lw=2)
        ax4.fill_between(f_all, -360, 180, where=(coh_all < COHERENCE_MIN), color="gray", alpha=0.3)
        ax4.set_title("Bode Phase Plot")
        ax4.set_xlabel("Frequency [Hz]")
        ax4.set_ylabel("Phase [deg]")
        ax4.set_ylim(-270, 90)
        ax4.axhline(-180, color="r", linestyle=":", alpha=0.7)
        ax4.grid(True, which="both", linestyle=":")
        if freq_metrics["f_cg"] and freq_metrics["phase_margin"] is not None:
            ax4.axvline(freq_metrics["f_cg"], color="r", linestyle="--")
            ax4.annotate(
                "",
                xy=(freq_metrics["f_cg"], -180),
                xytext=(freq_metrics["f_cg"], -180 + freq_metrics["phase_margin"]),
                arrowprops=dict(arrowstyle="<->", color="darkorange", lw=2),
            )
            ax4.text(
                freq_metrics["f_cg"] * 1.1,
                -180 + freq_metrics["phase_margin"] / 2,
                f"PM: {freq_metrics['phase_margin']:.1f}°",
                color="darkorange",
                fontweight="bold",
            )

    ax5 = axs[2, 0]
    if freq_metrics and "H_all" in freq_metrics:
        H_all = freq_metrics["H_all"]
        coh = freq_metrics["coherence_all"]
        valid = coh >= COHERENCE_MIN

        ax5.plot(np.real(H_all), np.imag(H_all), "b-", lw=1.0, alpha=0.3, label="Low Coherence")
        ax5.plot(np.real(H_all[valid]), np.imag(H_all[valid]), "b-", lw=2.5, label="High Coherence")
        ax5.plot(-1, 0, "rx", ms=10, mew=2, label="Critical Point")

        theta = np.linspace(0, 2 * np.pi, 100)
        ax5.plot(np.cos(theta), np.sin(theta), "k--", alpha=0.2)

        exclusion_circle = plt.Circle(
            (-1, 0), 0.5, color="r", fill=True, alpha=0.1, label="Exclusion Zone (M-Circle)"
        )
        ax5.add_patch(exclusion_circle)

        ax5.set_title("Nyquist Plot (Zoomed for Stability)")
        ax5.set_xlabel("Real")
        ax5.set_ylabel("Imaginary")
        ax5.grid(True, linestyle=":")
        ax5.axis("equal")
        ax5.set_xlim(-2.0, 1.0)
        ax5.set_ylim(-1.5, 1.5)
        ax5.legend(loc="upper right", fontsize=8)

    ax6 = axs[2, 1]
    if time_metrics:
        ax6.plot(time_metrics["t_norm"], time_metrics["y_norm"], "b-", label="Measured Response")
        ax6.plot(
            time_metrics["t_norm"],
            time_metrics["y_fit"],
            "r--",
            label=f"FOPDT Fit (R²={time_metrics['fopdt_r2']:.2f})",
        )
        ax6.set_title("System Identification (FOPDT Model Fit)")
        ax6.set_xlabel("Time since step [s]")
        ax6.set_ylabel(f"Velocity [{VELOCITY_UNIT}]")
        ax6.grid(True, linestyle=":")
        ax6.legend(loc="lower right")
        param_text = (
            f"Fitted Model parameters:\n"
            f"  K   = {time_metrics['fopdt_K']:.3f}\n"
            f"  tau = {time_metrics['fopdt_tau']:.3f} s\n"
            f"  L   = {time_metrics['fopdt_L']:.3f} s"
        )
        props = dict(boxstyle="round", facecolor="wheat", alpha=0.5)
        ax6.text(
            0.05,
            0.95,
            param_text,
            transform=ax6.transAxes,
            fontsize=10,
            verticalalignment="top",
            bbox=props,
        )

    ax7 = axs[3, 0]
    if freq_metrics and "coherence_all" in freq_metrics:
        ax7.semilogx(freq_metrics["f_all"], freq_metrics["coherence_all"], "g-", lw=2)
        ax7.set_title("Magnitude Squared Coherence")
        ax7.set_xlabel("Frequency [Hz]")
        ax7.set_ylabel("Coherence")
        ax7.axhline(COHERENCE_MIN, color="r", linestyle="--", label=f"Threshold ({COHERENCE_MIN})")
        ax7.grid(True, which="both", linestyle=":")
        ax7.set_ylim(0, 1.1)
        ax7.legend()

    ax8 = axs[3, 1]
    if freq_metrics and "Pxx" in freq_metrics:
        ax8.loglog(freq_metrics["f_all"], freq_metrics["Pxx"], "m-", label="Input PSD (Pxx)")
        ax8.loglog(freq_metrics["f_all"], freq_metrics["Pyy"], "c-", label="Output PSD (Pyy)")
        ax8.set_title("Power Spectral Density")
        ax8.set_xlabel("Frequency [Hz]")
        ax8.set_ylabel("Power / Hz")
        ax8.grid(True, linestyle=":")
        ax8.legend(loc="upper right", fontsize=8)

    ax9 = axs[4, 0]
    if "imu_accel" in data and len(data["imu_accel"]) > 0:
        ax9.plot(data["imu_accel"][:, 0], data["imu_accel"][:, 1], "g-", lw=1.5, alpha=0.7)
        ax9.axhline(5.0, color="r", linestyle="--", alpha=0.5, label="Acceleration Limit (5.0)")
        ax9.axhline(-5.0, color="r", linestyle="--", alpha=0.5)
        ax9.set_title("Raw IMU Linear Acceleration (X-Axis)")
        ax9.set_xlabel("Time [s]")
        ax9.set_ylabel("Accel [m/s^2]")
        ax9.grid(True, linestyle=":")
        ax9.legend(loc="upper right", fontsize=8)

    ax10 = axs[4, 1]
    if "imu_gyro" in data and len(data["imu_gyro"]) > 0:
        ax10.plot(data["imu_gyro"][:, 0], data["imu_gyro"][:, 1], "m-", lw=1.5, alpha=0.7)
        ax10.set_title("Raw IMU Angular Velocity (Z-Axis)")
        ax10.set_xlabel("Time [s]")
        ax10.set_ylabel("Gyro [rad/s]")
        ax10.grid(True, linestyle=":")

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(png_path, dpi=120)


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else None
    report_name = sys.argv[2] if len(sys.argv) > 2 else "control_report"
    output_dir = sys.argv[3] if len(sys.argv) > 3 else "."
    if not bag_path:
        print("Usage: ros2 run control_analysis analyze <bag_file_path> [report_name] [output_dir]")
        return 1
    return run_pro_analysis(bag_path, report_name=report_name, output_dir=output_dir)


if __name__ == "__main__":
    sys.exit(main())
