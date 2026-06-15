# Copyright 2026 Tatsukiyano
import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

def extract_bag_data(bag_path):
    storage_id = 'sqlite3' if glob.glob(os.path.join(bag_path, "*.db3")) else 'mcap'
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open bag: {e}")
        return None

    results = {'cmd': [], 'ekf': [], 'gt': [], 'effort': []}

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/cmd_vel_ext':
            msg = deserialize_message(data, TwistStamped)
            results['cmd'].append((t / 1e9, msg.twist.linear.x))
        elif topic == '/odometry/filtered':
            msg = deserialize_message(data, Odometry)
            # Filter out extreme initialization outliers
            vel = msg.twist.twist.linear.x
            if abs(vel) < 20.0:
                results['ekf'].append((t / 1e9, vel))
        elif topic == '/odom/ground_truth':
            msg = deserialize_message(data, Odometry)
            results['gt'].append((t / 1e9, msg.twist.twist.linear.x))
        elif topic == '/joint_states':
            msg = deserialize_message(data, JointState)
            if msg.effort:
                results['effort'].append((t / 1e9, sum(msg.effort)/len(msg.effort)))

    return {k: np.array(v) for k, v in results.items() if len(v) > 0}

def run_pro_analysis(bag_path):
    print(f"\n=== [PRO] Analyzing Control for: {bag_path} ===")
    data = extract_bag_data(bag_path)
    if not data or 'cmd' not in data or 'ekf' not in data:
        print("Error: Insufficient data found.")
        return

    # Normalize Time
    t0 = min(data['cmd'][0,0], data['ekf'][0,0])
    for k in data: data[k][:,0] -= t0

    step_idx = np.where(data['cmd'][:,1] >= 1.9)[0]
    t_step = data['cmd'][step_idx[0], 0] if len(step_idx) > 0 else 0

    plt.figure(figsize=(15, 10))

    # 1. Step Response
    plt.subplot(2, 2, 1)
    plt.step(data['cmd'][:,0], data['cmd'][:,1], 'r--', label='Command', where='post', alpha=0.5)
    plt.plot(data['ekf'][:,0], data['ekf'][:,1], 'b-', label='EKF Estimate', lw=2)
    plt.title('Time Domain: Step Response')
    plt.xlim(t_step - 1.0, t_step + 5.0); plt.grid(True); plt.legend()

    # 2. Effort Stability
    plt.subplot(2, 2, 2)
    if 'effort' in data:
        plt.plot(data['effort'][:,0], data['effort'][:,1], 'g-')
        plt.title('Control Effort (Torque)'); plt.xlim(t_step - 1.0, t_step + 5.0); plt.grid(True)

    # 3. Bode Plot
    chirp_mask = (data['cmd'][:,0] > 1.0) & (data['cmd'][:,0] < t_step - 1.0)
    if any(chirp_mask):
        t_c = data['ekf'][(data['ekf'][:,0] > 1.0) & (data['ekf'][:,0] < t_step - 1.0), 0]
        if len(t_c) > 100:
            u = np.interp(t_c, data['cmd'][:,0], data['cmd'][:,1])
            y = np.interp(t_c, data['ekf'][:,0], data['ekf'][:,1])
            fs = 1.0 / (t_c[1] - t_c[0])
            f, Pxy = signal.csd(u, y, fs, nperseg=256)
            f, Pxx = signal.welch(u, fs, nperseg=256)
            H = Pxy / Pxx
            valid = (f > 0.1) & (f < 15.0)

            plt.subplot(2, 2, 3)
            plt.semilogx(f[valid], 20*np.log10(np.abs(H[valid])), 'b-')
            plt.title('Bode Magnitude [dB]'); plt.grid(True, which='both')

            plt.subplot(2, 2, 4)
            plt.plot(np.real(H[valid]), np.imag(H[valid]), 'b-')
            plt.title('Nyquist Plot'); plt.grid(True); plt.axis('equal')

    plt.tight_layout()
    plt.savefig('control_report.png')
    print(">>> SUCCESS: Report generated <<<")

if __name__ == "__main__":
    if len(sys.argv) > 1: run_pro_analysis(sys.argv[1])
