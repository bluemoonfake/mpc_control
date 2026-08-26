# Reproducibility & Benchmark Run Directory Anatomy (REPRODUCIBILITY.md)

This document formalizes the validation run directory structure, metadata archiving, and benchmark reproduction rules.

---

## 1. Directory Anatomy (`validation_runs/`)

Every validated benchmark run is preserved in an isolated directory under `validation_runs/<mission_name>/<run_identifier>/`:

```text
validation_runs/
├── obstacle_slalom/
│   ├── run_04_corner_blend/
│   │   ├── tpmc_metrics.csv         # Raw high-frequency (50 Hz) telemetry CSV
│   │   ├── validation_report.json    # Complete JSON report with all 11 gate evaluations
│   │   ├── px4.log                   # PX4 SITL console output & preflight checks
│   │   └── ros.log                   # ROS 2 nodes stdout/stderr log
│   ├── run_05_repeat/
│   ├── run_06_repeat/
│   └── run_07_cpu_stress/
└── urban_canyon/
    ├── run_02_continuity/
    ├── run_03_repeat/
    ├── run_04_repeat/
    └── run_05_cpu_stress/
```

---

## 2. Archival Metadata Requirements for 100% Reproduction

To reproduce any historical benchmark flight identically, the following metadata elements are archived in `validation_report.json`:

1. **Git Commit & Dirty State**: SHA-1 commit hash of `mpc_control` and `PX4-Autopilot`.
2. **Environment Metadata**:
   - ROS Distribution (`Jazzy 24.04`)
   - Compiler & Flags (`GCC 11.4 Release`)
   - acados version & HPIPM version
   - Machine Specs: CPU model, core count, available RAM.
3. **Configuration Snapshots**:
   - `controller.yaml` parameter snapshot
   - Mission JSON waypoint coordinates, tolerances, and speeds.
4. **PX4 SITL Parameters**:
   - Airframe model (`gz_x500`)
   - Initial magnetometer calibration values (`CAL_MAG0_*OFF = 0.0`)
   - EKF2 estimator configuration.

---

## 3. Reproduction Command Workflow

To re-run and verify a historical validation dataset:
```bash
# 1. Reset magnetometer calibration offsets
python3 -c '
import struct
for fpath in ["/home/ubuntu/PX4_17/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson",
              "/home/ubuntu/PX4_17/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson"]:
    with open(fpath, "rb") as f: buf = bytearray(f.read())
    size = struct.unpack("<i", buf[:4])[0]; pos = 4
    while pos < size - 1:
        elem_type = buf[pos]; pos += 1
        end_str = buf.find(b"\x00", pos); name = buf[pos:end_str].decode("ascii"); pos = end_str + 1
        if elem_type == 0x01:
            if name in ["CAL_MAG0_XOFF", "CAL_MAG0_YOFF", "CAL_MAG0_ZOFF"]:
                struct.pack_into("<d", buf, pos, 0.0)
            pos += 8
        elif elem_type == 0x10: pos += 4
        elif elem_type == 0x02: s_len = struct.unpack("<i", buf[pos:pos+4])[0]; pos += 4 + s_len
    with open(fpath, "wb") as f: f.write(buf)
'

# 2. Run simulation and mission
SIM_LOG_XTERM=0 ROS_LAUNCH_ARGS="mission_file_path:=$PWD/config/missions/<mission>.json" make sim
make mission-execute
make stop

# 3. Generate validation report
make validation-report MISSION_JSON=config/missions/<mission>.json
```
