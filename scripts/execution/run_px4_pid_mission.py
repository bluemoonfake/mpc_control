#!/usr/bin/env python3
"""
Upload and execute benchmark_square.json on PX4 using native PID controller.
Supports:
  1. Uploading mission directly to PX4 via MAVLink (pymavlink).
  2. Generating standard QGroundControl .plan file for GUI upload.
  3. Auto-triggering PX4 native 'Auto: Mission' mode and arming.
"""

import json
import math
import sys
import time
import argparse
from pymavlink import mavutil

DEFAULT_LAT = 47.397742
DEFAULT_LON = 8.545594
DEFAULT_ALT = 488.0
EARTH_RADIUS = 6378137.0  # WGS-84 meters

def enu_to_gps(x_east, y_north, z_up, ref_lat, ref_lon, ref_alt):
    d_lat = y_north / EARTH_RADIUS
    d_lon = x_east / (EARTH_RADIUS * math.cos(math.radians(ref_lat)))
    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    alt = ref_alt + z_up
    return lat, lon, alt

def create_qgc_plan(mission_json_path, plan_output_path, ref_lat=DEFAULT_LAT, ref_lon=DEFAULT_LON, ref_alt=DEFAULT_ALT):
    with open(mission_json_path, 'r') as f:
        data = json.load(f)

    mission = data.get('mission', {})
    items = mission.get('items', [])
    defaults = mission.get('defaults', {})
    h_speed = defaults.get('horizontalVelocity', 4.0)

    qgc_items = []
    seq = 0

    for item in items:
        item_type = item.get('type')
        if item_type == 'takeoff':
            z = item.get('z', 10.0)
            lat, lon, _ = enu_to_gps(0.0, 0.0, z, ref_lat, ref_lon, ref_alt)
            qgc_items.append({
                "AMSLAltAboveTerrain": None,
                "Altitude": z,
                "AltitudeMode": 1,  # Relative to Home
                "autoContinue": True,
                "command": 22,  # MAV_CMD_NAV_TAKEOFF
                "doJumpId": seq + 1,
                "frame": 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                "params": [0, 0, 0, None, lat, lon, z],
                "type": "SimpleItem"
            })
            seq += 1
        elif item_type == 'navigation' and item.get('navigationType') == 'waypoint':
            x = item.get('x', 0.0)
            y = item.get('y', 0.0)
            z = item.get('z', 10.0)
            lat, lon, _ = enu_to_gps(x, y, z, ref_lat, ref_lon, ref_alt)
            qgc_items.append({
                "AMSLAltAboveTerrain": None,
                "Altitude": z,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": 16,  # MAV_CMD_NAV_WAYPOINT
                "doJumpId": seq + 1,
                "frame": 3,
                "params": [0, 2.0, 0, None, lat, lon, z],
                "type": "SimpleItem"
            })
            seq += 1
        elif item_type == 'land':
            lat, lon, _ = enu_to_gps(0.0, 0.0, 0.0, ref_lat, ref_lon, ref_alt)
            qgc_items.append({
                "AMSLAltAboveTerrain": None,
                "Altitude": 0,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": 21,  # MAV_CMD_NAV_LAND
                "doJumpId": seq + 1,
                "frame": 3,
                "params": [0, 0, 0, None, lat, lon, 0],
                "type": "SimpleItem"
            })
            seq += 1

    plan = {
        "fileType": "Plan",
        "version": 1,
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": h_speed,
            "hoverSpeed": h_speed,
            "firmwareType": 12,  # PX4
            "vehicleType": 2,  # Multi-Rotor
            "plannedHomePosition": [ref_lat, ref_lon, ref_alt],
            "items": qgc_items
        },
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2}
    }

    with open(plan_output_path, 'w') as f:
        json.dump(plan, f, indent=2)
    print(f"Generated QGC plan file: {plan_output_path}")
    return qgc_items

def upload_and_run_mavlink(mission_json_path, connection_str='udp:127.0.0.1:14550', auto_start=True):
    print(f"Connecting to PX4 via MAVLink at {connection_str}...")
    try:
        mav = mavutil.mavlink_connection(connection_str, source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=5)
        print(f"Heartbeat received from PX4 (system {mav.target_system})")
    except Exception as e:
        print(f"Could not connect via {connection_str}: {e}. Retrying on port 14540...")
        mav = mavutil.mavlink_connection('udp:127.0.0.1:14540', source_system=255, source_component=190)
        mav.wait_heartbeat(timeout=5)

    # Request home position
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    home_msg = mav.recv_match(type='HOME_POSITION', blocking=True, timeout=3)
    ref_lat = home_msg.latitude / 1e7 if home_msg else DEFAULT_LAT
    ref_lon = home_msg.longitude / 1e7 if home_msg else DEFAULT_LON
    ref_alt = home_msg.altitude / 1e3 if home_msg else DEFAULT_ALT
    print(f"Using Home Position: Lat={ref_lat:.6f}, Lon={ref_lon:.6f}, Alt={ref_alt:.1f}m")

    plan_path = mission_json_path.replace('.json', '.plan')
    qgc_items = create_qgc_plan(mission_json_path, plan_path, ref_lat, ref_lon, ref_alt)

    # Upload mission count
    count = len(qgc_items)
    print(f"Uploading {count} mission items to PX4...")
    mav.mav.mission_count_send(mav.target_system, mav.target_component, count, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    for i in range(count):
        req = mav.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
        if not req:
            print("Failed to receive mission request from PX4")
            return False
        seq = req.seq
        item = qgc_items[seq]
        params = item["params"]
        lat_int = int(params[4] * 1e7) if params[4] is not None else 0
        lon_int = int(params[5] * 1e7) if params[5] is not None else 0

        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component,
            seq,
            item["frame"],
            item["command"],
            1 if seq == 0 else 0,
            1,
            float(params[0] or 0),
            float(params[1] or 0),
            float(params[2] or 0),
            float(params[3] or 0) if params[3] is not None else 0.0,
            lat_int, lon_int,
            float(item["Altitude"]),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

    ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission successfully uploaded to PX4!")
    else:
        print(f"Mission upload failed or returned code: {ack.type if ack else 'timeout'}")
        return False

    if auto_start:
        print("Setting flight mode to AUTO: MISSION (PX4 native PID controller)...")
        # PX4 Custom mode AUTO: MISSION is main_mode=4 (AUTO), sub_mode=4 (MISSION)
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            (4 << 16) | (4 << 24)  # PX4_CUSTOM_MAIN_MODE_AUTO (4) | SUB_MODE_AUTO_MISSION (4)
        )
        time.sleep(0.5)

        print("Arming vehicle (force arm)...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 21196, 0, 0, 0, 0, 0
        )
        print("PX4 PID Mission started successfully!")
        print("Drone is now flying the benchmark course using native PX4 PID mc_pos_control.")

    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Upload and run benchmark mission on PX4 using native PID")
    parser.add_argument('--mission', default='config/missions/benchmark_square.json', help="Path to mission JSON")
    parser.add_argument('--connection', default='udp:127.0.0.1:14550', help="MAVLink connection URI")
    parser.add_argument('--plan-only', action='store_true', help="Only generate QGC .plan file without uploading")
    args = parser.parse_args()

    if args.plan_only:
        create_qgc_plan(args.mission, args.mission.replace('.json', '.plan'))
    else:
        upload_and_run_mavlink(args.mission, args.connection, auto_start=True)
