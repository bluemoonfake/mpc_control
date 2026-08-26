#!/usr/bin/env python3
"""
Mission Benchmark Log Comparison: PX4 Native PID vs. 3D Coupled MPC
---------------------------------------------------------------------
Parses two PX4 ULog files (one from standard PX4 PID mission, one from MPC mission),
computes side-by-side quantitative performance metrics, and plots overlay comparisons.

Usage:
    python3 scripts/compare_mission_logs.py <pid_log.ulg> <mpc_log.ulg> [--mission config/missions/benchmark_square.json] [--out comparison.png]
"""

import sys
import os
import json
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog


def extract_flight_data(ulog_path):
    """Extract position, velocity, attitude, torque/thrust, and active time from ULog."""
    ulog = ULog(ulog_path)
    
    # 1. Vehicle Local Position
    pos_data = ulog.get_dataset('vehicle_local_position').data
    t_pos = pos_data['timestamp'] * 1e-6
    x = pos_data['x']  # North in NED (or East/North depending on frame)
    y = pos_data['y']  # East in NED
    z = -pos_data['z'] # Up (meters)
    vx = pos_data['vx']
    vy = pos_data['vy']
    vz = -pos_data['vz']
    v_xy = np.sqrt(vx**2 + vy**2)
    heading = pos_data['heading']

    # 2. Control Mode (filter for active flight)
    try:
        ctrl_mode = ulog.get_dataset('vehicle_control_mode').data
        t_ctrl = ctrl_mode['timestamp'] * 1e-6
        # Active if offboard or auto enabled and armed
        is_active = (ctrl_mode['flag_control_offboard_enabled'] == 1) | (ctrl_mode['flag_control_auto_enabled'] == 1)
        if np.any(is_active):
            active_t_start = t_ctrl[np.where(is_active)[0][0]]
            active_t_end = t_ctrl[np.where(is_active)[0][-1]]
        else:
            active_t_start = t_pos[0]
            active_t_end = t_pos[-1]
    except Exception:
        active_t_start = t_pos[0]
        active_t_end = t_pos[-1]

    # Trim to active phase
    mask = (t_pos >= active_t_start) & (t_pos <= active_t_end)
    if not np.any(mask):
        mask = np.ones_like(t_pos, dtype=bool)

    t_rel = t_pos[mask] - t_pos[mask][0]
    
    # 3. Attitude & Rates
    try:
        att_data = ulog.get_dataset('vehicle_attitude').data
        t_att = att_data['timestamp'] * 1e-6
        q0 = att_data['q[0]']
        q1 = att_data['q[1]']
        q2 = att_data['q[2]']
        q3 = att_data['q[3]']
        # True inclination/tilt angle relative to vertical Z axis:
        r33 = 1.0 - 2.0 * (q1**2 + q2**2)
        tilt = np.degrees(np.arccos(np.clip(r33, -1.0, 1.0)))
        tilt_interp = np.interp(t_pos[mask], t_att, tilt)
    except Exception:
        tilt_interp = np.zeros_like(t_rel)

    # 4. Thrust / Actuator data
    try:
        thrust_data = ulog.get_dataset('vehicle_thrust_setpoint').data
        t_thrust = thrust_data['timestamp'] * 1e-6
        thrust_z = -thrust_data['xyz[2]']
        thrust_interp = np.interp(t_pos[mask], t_thrust, thrust_z)
    except Exception:
        thrust_interp = np.zeros_like(t_rel)

    # 5. Torque Setpoint
    try:
        torque_data = ulog.get_dataset('vehicle_torque_setpoint').data
        t_torque = torque_data['timestamp'] * 1e-6
        torque_norm = np.sqrt(torque_data['xyz[0]']**2 + torque_data['xyz[1]']**2 + torque_data['xyz[2]']**2)
        torque_interp = np.interp(t_pos[mask], t_torque, torque_norm)
    except Exception:
        torque_interp = np.zeros_like(t_rel)

    return {
        't': t_rel,
        'x': x[mask],
        'y': y[mask],
        'z': z[mask],
        'vx': vx[mask],
        'vy': vy[mask],
        'vz': vz[mask],
        'v_xy': v_xy[mask],
        'heading': heading[mask],
        'tilt': tilt_interp,
        'thrust': thrust_interp,
        'torque': torque_interp,
        'duration': t_rel[-1] if len(t_rel) > 0 else 0.0,
    }


def compute_metrics(data, name):
    """Compute quantitative performance metrics."""
    duration = data['duration']
    v_xy = data['v_xy']
    v_mean = np.mean(v_xy)
    v_max = np.max(v_xy)
    
    # Acceleration / Jerk estimation
    if len(data['t']) > 2:
        t = data['t']
        # Filter strictly increasing time points to avoid divide by zero
        valid_idx = np.where(np.diff(t) > 1e-4)[0]
        if len(valid_idx) > 2:
            t_sub = t[valid_idx]
            vx_sub = data['vx'][valid_idx]
            vy_sub = data['vy'][valid_idx]
            vz_sub = data['vz'][valid_idx]
            ax = np.gradient(vx_sub, t_sub)
            ay = np.gradient(vy_sub, t_sub)
            az = np.gradient(vz_sub, t_sub)
            a_norm = np.sqrt(ax**2 + ay**2 + az**2)
            jx = np.gradient(ax, t_sub)
            jy = np.gradient(ay, t_sub)
            jz = np.gradient(az, t_sub)
            j_norm = np.sqrt(jx**2 + jy**2 + jz**2)
            jerk_rms = float(np.sqrt(np.mean(j_norm**2)))
            acc_rms = float(np.sqrt(np.mean(a_norm**2)))
        else:
            jerk_rms = 0.0
            acc_rms = 0.0
    else:
        jerk_rms = 0.0
        acc_rms = 0.0

    # Control effort
    thrust_rms = np.sqrt(np.mean(data['thrust']**2)) if len(data['thrust']) > 0 else 0.0
    torque_rms = np.sqrt(np.mean(data['torque']**2)) if len(data['torque']) > 0 else 0.0
    tilt_max = np.max(data['tilt']) if len(data['tilt']) > 0 else 0.0
    tilt_mean = np.mean(data['tilt']) if len(data['tilt']) > 0 else 0.0

    # Altitude stability (std deviation around median active altitude)
    z_std = np.std(data['z'])

    return {
        'name': name,
        'duration_s': duration,
        'v_mean_m_s': v_mean,
        'v_max_m_s': v_max,
        'acc_rms_m_s2': acc_rms,
        'jerk_rms_m_s3': jerk_rms,
        'tilt_mean_deg': tilt_mean,
        'tilt_max_deg': tilt_max,
        'thrust_rms': thrust_rms,
        'torque_rms': torque_rms,
        'z_std_m': z_std,
    }


def load_mission_waypoints(mission_path):
    """Extract ENU waypoints from mission JSON if available."""
    if not mission_path or not os.path.exists(mission_path):
        return None
    try:
        with open(mission_path, 'r') as f:
            data = json.load(f)
        wps = []
        for item in data.get('mission', {}).get('items', []):
            if item.get('type') == 'navigation' and item.get('navigationType') == 'waypoint':
                wps.append([item.get('x', 0.0), item.get('y', 0.0), item.get('z', 0.0)])
            elif item.get('type') == 'takeoff':
                wps.append([0.0, 0.0, item.get('z', 10.0)])
        return np.array(wps) if len(wps) > 0 else None
    except Exception as e:
        print(f"Warning: could not parse mission file {mission_path}: {e}")
        return None


def plot_comparison(pid_data, mpc_data, waypoints, output_png):
    """Generate multi-panel comparison visualization."""
    fig = plt.figure(figsize=(18, 12), dpi=150)
    fig.suptitle('PX4 Native PID vs. 3D Coupled MPC — Mission Benchmark Comparison', fontsize=16, fontweight='bold')

    # Color definitions
    c_pid = '#E63946' # Red
    c_mpc = '#1D3557' # Deep Navy / Cyan accent
    c_wp = '#2A9D8F'  # Teal

    # 1. 2D Horizontal Trajectory
    ax1 = fig.add_subplot(2, 3, 1)
    ax1.set_title('2D Trajectory (X-Y Path)', fontweight='bold')
    if waypoints is not None and len(waypoints) > 0:
        ax1.plot(waypoints[:, 0], waypoints[:, 1], 'o--', color=c_wp, label='Mission Waypoints', markersize=7, alpha=0.7)
    ax1.plot(pid_data['x'], pid_data['y'], label='PX4 PID', color=c_pid, linewidth=2, linestyle='--')
    ax1.plot(mpc_data['x'], mpc_data['y'], label='MPC Controller', color=c_mpc, linewidth=2.2)
    ax1.set_xlabel('East / X [m]')
    ax1.set_ylabel('North / Y [m]')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')
    ax1.axis('equal')

    # 2. 3D Trajectory
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    ax2.set_title('3D Trajectory (X-Y-Z)', fontweight='bold')
    if waypoints is not None and len(waypoints) > 0:
        ax2.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 'o--', color=c_wp, label='Waypoints', markersize=6)
    ax2.plot(pid_data['x'], pid_data['y'], pid_data['z'], label='PID', color=c_pid, linewidth=2, linestyle='--')
    ax2.plot(mpc_data['x'], mpc_data['y'], mpc_data['z'], label='MPC', color=c_mpc, linewidth=2.2)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_zlabel('Altitude Z [m]')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best')

    # 3. Horizontal Speed Profile V_xy(t)
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.set_title('Horizontal Speed Profile $V_{xy}(t)$', fontweight='bold')
    ax3.plot(pid_data['t'], pid_data['v_xy'], label='PX4 PID (Stop-and-Go)', color=c_pid, linewidth=1.8, linestyle='--')
    ax3.plot(mpc_data['t'], mpc_data['v_xy'], label='MPC (Continuous Cornering)', color=c_mpc, linewidth=2.0)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Speed [m/s]')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best')

    # 4. Altitude Profile Z(t)
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.set_title('Altitude Tracking $Z(t)$', fontweight='bold')
    ax4.plot(pid_data['t'], pid_data['z'], label='PX4 PID', color=c_pid, linewidth=1.8, linestyle='--')
    ax4.plot(mpc_data['t'], mpc_data['z'], label='MPC', color=c_mpc, linewidth=2.0)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Altitude [m]')
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='best')

    # 5. Tilt Angle (Roll/Pitch Magnitude)
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.set_title('Vehicle Tilt Angle $\\theta(t)$', fontweight='bold')
    ax5.plot(pid_data['t'], pid_data['tilt'], label='PX4 PID', color=c_pid, linewidth=1.8, linestyle='--')
    ax5.plot(mpc_data['t'], mpc_data['tilt'], label='MPC', color=c_mpc, linewidth=2.0)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Tilt [deg]')
    ax5.grid(True, alpha=0.3)
    ax5.legend(loc='best')

    # 6. Control Effort (Normalized Torque & Thrust)
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.set_title('Body Torque Effort $\\|\\tau(t)\\|$', fontweight='bold')
    ax6.plot(pid_data['t'], pid_data['torque'], label='PX4 PID Torque', color=c_pid, linewidth=1.8, linestyle='--')
    ax6.plot(mpc_data['t'], mpc_data['torque'], label='MPC SO(3) Torque', color=c_mpc, linewidth=2.0)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Torque Norm [norm. units]')
    ax6.grid(True, alpha=0.3)
    ax6.legend(loc='best')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(output_png, bbox_inches='tight')
    print(f"Comparison plot saved to: {output_png}")


def main():
    parser = argparse.ArgumentParser(description='Benchmark Comparison: PX4 Native PID vs. MPC Controller')
    parser.add_argument('pid_log', help='Path to PX4 PID ULog file')
    parser.add_argument('mpc_log', help='Path to MPC ULog file')
    parser.add_argument('--mission', default='config/missions/benchmark_square.json', help='Path to mission JSON file')
    parser.add_argument('--out', default='mission_benchmark_comparison.png', help='Output PNG path')
    args = parser.parse_args()

    print(f"\n=======================================================")
    print(f" PX4 MISSION BENCHMARK COMPARISON TOOL")
    print(f"=======================================================")
    print(f" PID Log:  {args.pid_log}")
    print(f" MPC Log:  {args.mpc_log}")
    print(f" Mission:  {args.mission}")

    pid_data = extract_flight_data(args.pid_log)
    mpc_data = extract_flight_data(args.mpc_log)
    waypoints = load_mission_waypoints(args.mission)

    m_pid = compute_metrics(pid_data, "PX4 Native PID")
    m_mpc = compute_metrics(mpc_data, "3D Coupled MPC")

    time_diff_pct = (m_mpc['duration_s'] - m_pid['duration_s']) / max(m_pid['duration_s'], 1e-3) * 100.0
    v_diff_pct = (m_mpc['v_mean_m_s'] - m_pid['v_mean_m_s']) / max(m_pid['v_mean_m_s'], 1e-3) * 100.0

    print(f"\n--- QUANTITATIVE BENCHMARK METRICS ---")
    print(f"{'Metric':<35} | {'PX4 Native PID':<16} | {'3D Coupled MPC':<16} | {'Advantage':<15}")
    print(f"{'-'*35}-+-{'-'*16}-+-{'-'*16}-+-{'-'*15}")
    print(f"{'Mission Completion Time [s]':<35} | {m_pid['duration_s']:<16.2f} | {m_mpc['duration_s']:<16.2f} | {time_diff_pct:<+14.1f}%")
    print(f"{'Mean Speed [m/s]':<35} | {m_pid['v_mean_m_s']:<16.2f} | {m_mpc['v_mean_m_s']:<16.2f} | {v_diff_pct:<+14.1f}%")
    print(f"{'Max Speed [m/s]':<35} | {m_pid['v_max_m_s']:<16.2f} | {m_mpc['v_max_m_s']:<16.2f} |")
    print(f"{'RMS Acceleration [m/s^2]':<35} | {m_pid['acc_rms_m_s2']:<16.3f} | {m_mpc['acc_rms_m_s2']:<16.3f} |")
    print(f"{'RMS Jerk Smoothness [m/s^3]':<35} | {m_pid['jerk_rms_m_s3']:<16.3f} | {m_mpc['jerk_rms_m_s3']:<16.3f} |")
    print(f"{'Max Tilt Angle [deg]':<35} | {m_pid['tilt_max_deg']:<16.1f} | {m_mpc['tilt_max_deg']:<16.1f} |")
    print(f"{'Mean Tilt Angle [deg]':<35} | {m_pid['tilt_mean_deg']:<16.1f} | {m_mpc['tilt_mean_deg']:<16.1f} |")
    print(f"{'RMS Torque Effort':<35} | {m_pid['torque_rms']:<16.4f} | {m_mpc['torque_rms']:<16.4f} |")
    print(f"{'Altitude Std Dev [m]':<35} | {m_pid['z_std_m']:<16.3f} | {m_mpc['z_std_m']:<16.3f} |")
    print(f"{'-'*35}-+-{'-'*16}-+-{'-'*16}-+-{'-'*15}\n")

    plot_comparison(pid_data, mpc_data, waypoints, args.out)


if __name__ == '__main__':
    main()
