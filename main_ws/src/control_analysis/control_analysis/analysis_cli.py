# Copyright 2026 Tatsukiyano
import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import minimize
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


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
        "cmd": [],  # Raw ext command
        "cmd_stab": [],  # Stabilized command
        "ekf": [],  # EKF estimate
        "gt": [],  # Ground truth
        "effort": [],  # Motor torque/effort
    }

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == "/cmd_vel_ext":
            msg = deserialize_message(data, TwistStamped)
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if t_msg < 1e-3:
                t_msg = t / 1e9
            results["cmd"].append((t_msg, msg.twist.linear.x))
        elif topic == "/mecanum_drive_controller/reference":
            msg = deserialize_message(data, TwistStamped)
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if t_msg < 1e-3:
                t_msg = t / 1e9
            results["cmd_stab"].append((t_msg, msg.twist.linear.x))
        elif topic == "/odometry/filtered":
            msg = deserialize_message(data, Odometry)
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if t_msg < 1e-3:
                t_msg = t / 1e9
            vel = msg.twist.twist.linear.x
            if abs(vel) < 20.0:  # Filter outliers
                results["ekf"].append((t_msg, vel))
        elif topic == "/odom/ground_truth":
            msg = deserialize_message(data, Odometry)
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if t_msg < 1e-3:
                t_msg = t / 1e9
            results["gt"].append((t_msg, msg.twist.twist.linear.x))
        elif topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if t_msg < 1e-3:
                t_msg = t / 1e9
            if msg.effort:
                results["effort"].append((t_msg, np.mean(np.abs(msg.effort))))

    return {k: np.array(v) for k, v in results.items() if len(v) > 0}


def analyze_step_response(t_cmd, u_cmd, t_resp, y_resp, t_step):
    # Find indices for step window
    t_end = t_cmd[-1]
    duration = t_end - t_step

    mask_resp = (t_resp >= t_step) & (t_resp <= t_end)
    t_step_data = t_resp[mask_resp]
    y_step_data = y_resp[mask_resp]

    mask_cmd = (t_cmd >= t_step) & (t_cmd <= t_end)
    t_cmd_data = t_cmd[mask_cmd]
    u_cmd_data = u_cmd[mask_cmd]

    if len(t_step_data) < 10 or len(u_cmd_data) < 2:
        return None

    # Baseline & Target definitions
    u_baseline = u_cmd[t_cmd < t_step][-1] if any(t_cmd < t_step) else 0.0
    u_target = u_cmd_data[0]
    u_step = u_target - u_baseline

    t_norm = t_step_data - t_step
    y_norm = y_step_data - u_baseline

    # Average last 20% of response to estimate steady state
    ss_mask = t_norm > (0.8 * duration)
    y_ss = np.mean(y_norm[ss_mask]) if any(ss_mask) else y_norm[-1]

    # Process Gain Kp
    K_p = y_ss / u_step if abs(u_step) > 1e-5 else 0.0

    # Overshoot Mp (%)
    if u_step > 0:
        y_peak = np.max(y_norm)
        idx_peak = np.argmax(y_norm)
        overshoot = max(0.0, (y_peak - y_ss) / abs(y_ss)) * 100.0 if abs(y_ss) > 1e-5 else 0.0
    else:
        y_peak = np.min(y_norm)
        idx_peak = np.argmin(y_norm)
        overshoot = max(0.0, (y_ss - y_peak) / abs(y_ss)) * 100.0 if abs(y_ss) > 1e-5 else 0.0

    t_peak = t_norm[idx_peak]

    # Rise Time tr (10% to 90% of steady-state change)
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

    # Settling Time ts (2% band of steady state change)
    err = np.abs(y_norm - y_ss)
    thresh_2 = 0.02 * abs(y_ss)
    idx_out_2 = np.where(err > thresh_2)[0]
    settling_time_2 = t_norm[idx_out_2[-1]] if len(idx_out_2) > 0 else 0.0

    # FOPDT (First-Order Plus Dead Time) Fit: G(s) = K / (tau * s + 1) * e^(-L*s)
    def fopdt_response(t, K, tau, L):
        resp = np.zeros_like(t)
        active = t >= L
        resp[active] = K * u_step * (1.0 - np.exp(-(t[active] - L) / tau))
        return resp

    def loss_func(params):
        K, tau, L = params
        y_fit = fopdt_response(t_norm, K, tau, L)
        return np.sum((y_norm - y_fit) ** 2)

    # Initial guesses
    val_63 = 0.632 * y_ss
    idx_63 = np.where(y_norm >= val_63 if u_step > 0 else y_norm <= val_63)[0]
    t_63 = t_norm[idx_63[0]] if len(idx_63) > 0 else 0.5

    K_guess = K_p
    tau_guess = max(0.01, t_63)
    L_guess = 0.05

    res = minimize(
        loss_func,
        [K_guess, tau_guess, L_guess],
        bounds=[(0.01, 10.0), (0.01, 5.0), (0.0, 2.0)],
        method="L-BFGS-B",
    )
    K_fit, tau_fit, L_fit = res.x

    # Fit quality (R^2)
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


def analyze_frequency_response(t_cmd, u_cmd, t_resp, y_resp, chirp_t0, chirp_t1):
    fs = 100.0
    dt = 1.0 / fs
    duration = chirp_t1 - chirp_t0
    if duration <= 0.0 or duration > 100.0:
        print(
            f"Error: Invalid chirp duration {duration:.2f}s (must be between 0 and 100s). Check time synchronization."
        )
        return None
    t_uniform = np.arange(chirp_t0, chirp_t1, dt)
    u_uniform = np.interp(t_uniform, t_cmd, u_cmd)
    y_uniform = np.interp(t_uniform, t_resp, y_resp)

    # Detrend to remove DC bias
    u_detrend = signal.detrend(u_uniform)
    y_detrend = signal.detrend(y_uniform)

    nperseg = min(512, len(t_uniform) // 2)
    if nperseg < 64:
        return None

    f, Pxx = signal.welch(u_detrend, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    f, Pyy = signal.welch(y_detrend, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    f, Pxy = signal.csd(u_detrend, y_detrend, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)

    H = Pxy / Pxx
    coherence = np.abs(Pxy) ** 2 / (Pxx * Pyy + 1e-12)

    valid = (f >= 0.1) & (f <= 20.0)
    f_valid = f[valid]
    H_valid = H[valid]
    coh_valid = coherence[valid]

    mag_db = 20 * np.log10(np.abs(H_valid) + 1e-12)
    phase_deg = np.rad2deg(np.unwrap(np.angle(H_valid)))

    # Gain Crossover Frequency (where magnitude crosses 0 dB)
    f_cg, phase_margin = None, None
    mag_diff = mag_db - 0.0
    idx_cg = np.where(np.diff(np.sign(mag_diff)))[0]
    if len(idx_cg) > 0:
        i = idx_cg[0]
        f_cg = f_valid[i] + (f_valid[i + 1] - f_valid[i]) * (0.0 - mag_db[i]) / (
            mag_db[i + 1] - mag_db[i]
        )
        phase_at_cg = np.interp(f_cg, f_valid, phase_deg)
        phase_margin = (phase_at_cg - (-180.0) + 180.0) % 360.0 - 180.0

    # Phase Crossover Frequency (where phase crosses -180 deg)
    f_cp, gain_margin = None, None
    phase_diff = phase_deg - (-180.0)
    idx_cp = np.where(np.diff(np.sign(phase_diff)))[0]
    if len(idx_cp) > 0:
        i = idx_cp[0]
        f_cp = f_valid[i] + (f_valid[i + 1] - f_valid[i]) * (-180.0 - phase_deg[i]) / (
            phase_deg[i + 1] - phase_deg[i]
        )
        mag_at_cp = np.interp(f_cp, f_valid, np.abs(H_valid))
        gain_margin = -20 * np.log10(mag_at_cp + 1e-12)

    return {
        "freq": f_valid,
        "H": H_valid,
        "mag_db": mag_db,
        "phase_deg": phase_deg,
        "coherence": coh_valid,
        "f_cg": f_cg,
        "phase_margin": phase_margin,
        "f_cp": f_cp,
        "gain_margin": gain_margin,
    }


def print_ascii_report(time_metrics, freq_metrics):
    print("\n" + "=" * 60)
    print("           CONTROL SYSTEM IDENTIFICATION REPORT (PRO)")
    print("=" * 60)

    if time_metrics:
        print("\n[Time Domain: Step Response Metrics]")
        print(f"  Step Command Jump      : {time_metrics['u_step']:.2f} rad/s")
        print(f"  Steady-State Value     : {time_metrics['y_norm'][-1]:.2f} rad/s")
        print(f"  Steady-State Gain (Kp) : {time_metrics['K_p']:.4f}")
        print(f"  Steady-State Error     : {time_metrics['ss_error']:.4f} rad/s")
        print(f"  Overshoot (Mp)         : {time_metrics['overshoot']:.2f} %")
        print(f"  Peak Time (tp)         : {time_metrics['t_peak']:.3f} s")
        print(
            f"  Rise Time (tr, 10-90%) : {time_metrics['rise_time']:.3f} s"
            if time_metrics["rise_time"]
            else "  Rise Time (tr, 10-90%) : N/A"
        )
        print(f"  Settling Time (ts, 2%) : {time_metrics['settling_time_2']:.3f} s")
        print("\n[Estimated System Transfer Function (FOPDT)]")
        print(f"  Model                  : G(s) = K * exp(-L*s) / (tau*s + 1)")
        print(f"  Process Gain (K)       : {time_metrics['fopdt_K']:.4f}")
        print(f"  Time Constant (tau)    : {time_metrics['fopdt_tau']:.4f} s")
        print(f"  Dead Time / Delay (L)  : {time_metrics['fopdt_L']:.4f} s")
        print(f"  Model Accuracy (R^2)   : {time_metrics['fopdt_r2'] * 100:.2f} %")

    if freq_metrics:
        print("\n[Frequency Domain: Dynamic Margins]")
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


def run_pro_analysis(bag_path):
    print(f"\n=== [PRO] Starting High-End Control Analysis for: {bag_path} ===")
    data = extract_bag_data(bag_path)
    if not data or "cmd" not in data or "ekf" not in data:
        print("Error: Insufficient cmd or EKF data in bag file.")
        return

    # Sync and normalize times relative to earliest log timestamp
    t0 = min(data["cmd"][0, 0], data["ekf"][0, 0])
    for k in data:
        data[k][:, 0] -= t0

    # Segment bag into Chirp and Step sequences using command log
    step_indices = np.where(data["cmd"][:, 1] >= 1.9)[0]
    if len(step_indices) == 0:
        # Fallback if step is not 2.0 amplitude
        diffs = np.abs(np.diff(data["cmd"][:, 1]))
        step_indices = np.where(diffs > 0.5)[0]

    t_step = (
        data["cmd"][step_indices[0], 0] if len(step_indices) > 0 else (data["cmd"][-1, 0] - 5.0)
    )

    # Perform time domain analysis
    time_metrics = analyze_step_response(
        data["cmd"][:, 0], data["cmd"][:, 1], data["ekf"][:, 0], data["ekf"][:, 1], t_step
    )

    # Perform frequency domain analysis on chirp region [1.0s, t_step - 1.0s]
    freq_metrics = None
    if t_step > 5.0:
        freq_metrics = analyze_frequency_response(
            data["cmd"][:, 0],
            data["cmd"][:, 1],
            data["ekf"][:, 0],
            data["ekf"][:, 1],
            1.0,
            t_step - 1.0,
        )

    # Print clean text summary to command line
    print_ascii_report(time_metrics, freq_metrics)

    # Generate Professional Multi-Panel Control Dashboard
    fig, axs = plt.subplots(3, 2, figsize=(16, 12))
    plt.suptitle(
        f"Control Engineering Performance Dashboard\nBag: {os.path.basename(bag_path)}",
        fontsize=16,
        fontweight="bold",
    )

    # Panel 1: Time Domain Response
    ax1 = axs[0, 0]
    ax1.step(data["cmd"][:, 0], data["cmd"][:, 1], "r--", label="Command", where="post", alpha=0.6)
    if "cmd_stab" in data:
        ax1.step(
            data["cmd_stab"][:, 0],
            data["cmd_stab"][:, 1],
            "m--",
            label="Stab Command",
            where="post",
            alpha=0.4,
        )
    ax1.plot(data["ekf"][:, 0], data["ekf"][:, 1], "b-", label="EKF Estimate", lw=2)
    if "gt" in data:
        ax1.plot(data["gt"][:, 0], data["gt"][:, 1], "g-", label="Ground Truth", alpha=0.7)
    ax1.set_title("Step Response (Time Domain)")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Velocity [rad/s]")
    ax1.set_xlim(t_step - 1.0, data["cmd"][-1, 0])
    ax1.grid(True, linestyle=":")
    ax1.legend(loc="lower right")

    if time_metrics:
        # Draw metric indicators on Step Response
        ax1.axvline(
            t_step + time_metrics["settling_time_2"], color="orange", linestyle=":", label="ts (2%)"
        )
        if time_metrics["rise_time"]:
            ax1.axvspan(
                t_step,
                t_step + time_metrics["rise_time"],
                color="gray",
                alpha=0.15,
                label="tr (10-90%)",
            )
        ax1.axhline(time_metrics["y_ss"], color="b", linestyle="-.", alpha=0.5)

    # Panel 2: Tracking Error and Control Effort
    ax2 = axs[0, 1]
    if time_metrics:
        # Interpolate cmd to match EKF timing
        cmd_interp = np.interp(data["ekf"][:, 0], data["cmd"][:, 0], data["cmd"][:, 1])
        error = cmd_interp - data["ekf"][:, 1]
        ax2.plot(data["ekf"][:, 0], error, "r-", label="Velocity Error")
        ax2.set_ylabel("Error [rad/s]", color="r")
        ax2.tick_params(axis="y", labelcolor="r")
        ax2.set_xlim(t_step - 1.0, data["cmd"][-1, 0])
        ax2.grid(True, linestyle=":")

        # Overlay joint efforts if present
        if "effort" in data:
            ax2_right = ax2.twinx()
            ax2_right.plot(
                data["effort"][:, 0], data["effort"][:, 1], "g-", alpha=0.6, label="Mean Torque"
            )
            ax2_right.set_ylabel("Actuator Effort [Nm]", color="g")
            ax2_right.tick_params(axis="y", labelcolor="g")

        ax2.set_title("Control Accuracy and Effort")
        ax2.set_xlabel("Time [s]")

    # Panel 3: Bode Magnitude
    ax3 = axs[1, 0]
    if freq_metrics:
        ax3.semilogx(freq_metrics["freq"], freq_metrics["mag_db"], "b-", lw=2)
        ax3.set_title("Bode Magnitude Plot")
        ax3.set_xlabel("Frequency [Hz]")
        ax3.set_ylabel("Magnitude [dB]")
        ax3.grid(True, which="both", linestyle=":")
        if freq_metrics["f_cg"]:
            ax3.axvline(
                freq_metrics["f_cg"],
                color="r",
                linestyle="--",
                label=f"Crossover: {freq_metrics['f_cg']:.2f}Hz",
            )
            ax3.legend()

    # Panel 4: Bode Phase
    ax4 = axs[1, 1]
    if freq_metrics:
        ax4.semilogx(freq_metrics["freq"], freq_metrics["phase_deg"], "b-", lw=2)
        ax4.set_title("Bode Phase Plot")
        ax4.set_xlabel("Frequency [Hz]")
        ax4.set_ylabel("Phase [deg]")
        ax4.axhline(-180, color="r", linestyle=":", alpha=0.7)
        ax4.grid(True, which="both", linestyle=":")
        if freq_metrics["f_cg"] and freq_metrics["phase_margin"] is not None:
            ax4.axvline(freq_metrics["f_cg"], color="r", linestyle="--")
            # Mark PM
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

    # Panel 5: Nyquist Diagram
    ax5 = axs[2, 0]
    if freq_metrics:
        real_part = np.real(freq_metrics["H"])
        imag_part = np.imag(freq_metrics["H"])
        ax5.plot(real_part, imag_part, "b-", lw=2)
        ax5.plot(-1, 0, "rx", ms=10, mew=2, label="Critical Point (-1,0)")

        # Unit Circle
        theta = np.linspace(0, 2 * np.pi, 100)
        ax5.plot(np.cos(theta), np.sin(theta), "k--", alpha=0.3)

        ax5.set_title("Nyquist Plot")
        ax5.set_xlabel("Real")
        ax5.set_ylabel("Imaginary")
        ax5.grid(True, linestyle=":")
        ax5.axis("equal")
        ax5.set_xlim(-2.0, 2.0)
        ax5.set_ylim(-2.0, 2.0)
        ax5.legend()

    # Panel 6: FOPDT Model Fitting and Coherence
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
        ax6.set_ylabel("Velocity [rad/s]")
        ax6.grid(True, linestyle=":")
        ax6.legend(loc="lower right")

        # Add text box with estimated parameters
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

    plt.tight_layout()
    plt.savefig("control_report.png", dpi=150)
    print(">>> SUCCESS: control_report.png generated <<<")


def main():
    if len(sys.argv) > 1:
        run_pro_analysis(sys.argv[1])
    else:
        print("Usage: ros2 run control_analysis analyze <bag_file_path>")


if __name__ == "__main__":
    main()
