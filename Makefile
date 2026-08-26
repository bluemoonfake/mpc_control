# Compact simulator orchestration for the PX4 torque/thrust Offboard path.
#
# The Makefile does not arm PX4 and does not change PX4 failsafe parameters.
# PX4 and px4_msgs remain external dependencies of this source-only branch.

SHELL := /bin/bash

ROS_SETUP ?= /opt/ros/jazzy/setup.bash
PX4_DIR ?= $(firstword $(wildcard $(CURDIR)/third_party/PX4-Autopilot $(HOME)/PX4_17/PX4-Autopilot))
PX4_MSGS_SETUP ?= $(firstword $(wildcard $(CURDIR)/install/px4_msgs/local_setup.bash $(CURDIR)/install/ros_px4_msgs/local_setup.bash /tmp/mpc_controller_px4_msgs_install.*/local_setup.bash))
ROS_WORKSPACE_SETUP ?= $(CURDIR)/install/local_setup.bash
PX4_TARGET ?= px4_sitl
PX4_SIM ?= gz_x500
PX4_SYS_ID ?= 2
PX4_SIM_MODEL_INSTANCE ?= 0
# Keep the ROS 2 and PX4 uXRCE-DDS participants on the requested domain.
ROS_DOMAIN_ID ?= 42
PX4_BUILD_DIR ?= $(PX4_DIR)/build/px4_sitl_default
PX4_EXPECTED_COMMIT ?= 0b6e4687defb353a34201951809efd3f0040a9ba
GZ_EXPECTED_VERSION ?= 8.11.0

# Hardware-in-the-loop defaults. HIL_DEVICE may be overridden with a concrete
# /dev/ttyACM* path; leaving it empty makes the recipe prefer a stable PX4
# /dev/serial/by-id symlink and fall back to the first ttyACM device.
HIL_BOARD ?= px4_fmu-v6x
HIL_TARGET ?= px4_fmu-v6x_default
HIL_DEVICE ?=
HIL_BAUD ?= 921600
HIL_RATE_HZ ?= 250
HIL_HEADLESS ?= 0
HIL_CONFIG := $(PX4_DIR)/boards/px4/fmu-v6x/default.px4board
JMAVSIM_RUNNER := $(PX4_DIR)/Tools/simulation/jmavsim/jmavsim_run.sh

DDS_AGENT ?= MicroXRCEAgent
DDS_TRANSPORT ?= udp4
DDS_PORT ?= 8888
PX4_UXRCE_DDS_PORT ?= $(DDS_PORT)

ROS_PACKAGE ?= mpc_controller
ROS_LAUNCH ?= mpc_offboard.launch.py
ROS_LAUNCH_ARGS ?=
ROS_BUILD_ARGS ?= --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# The package contains several Eigen-heavy translation units. Building all of
# them with the host-default job count can exhaust RAM and kill cc1plus.
ROS_BUILD_PARALLEL_WORKERS ?= 1

SIM_RUNTIME_DIR ?= /tmp/mpc_controller_sim
SIM_DDS_DISCOVERY_TIMEOUT_SECONDS ?= 45
PX4_LOG := $(SIM_RUNTIME_DIR)/px4.log
DDS_LOG := $(SIM_RUNTIME_DIR)/dds.log
ROS_LOG := $(SIM_RUNTIME_DIR)/ros.log
TMPC_METRICS_LOG := $(SIM_RUNTIME_DIR)/tpmc_metrics.log
GZ_GUI_LOG := $(SIM_RUNTIME_DIR)/gazebo_gui.log
HIL_LOG := $(SIM_RUNTIME_DIR)/jmavsim_hil.log
PX4_PID := $(SIM_RUNTIME_DIR)/px4.pid
DDS_PID := $(SIM_RUNTIME_DIR)/dds.pid
ROS_PID := $(SIM_RUNTIME_DIR)/ros.pid
ROS_LOCK := $(SIM_RUNTIME_DIR)/ros.lock
TMPC_METRICS_PID := $(SIM_RUNTIME_DIR)/tpmc_metrics.pid
GZ_GUI_PID := $(SIM_RUNTIME_DIR)/gazebo_gui.pid
HIL_PID := $(SIM_RUNTIME_DIR)/jmavsim_hil.pid

# The VS Code Snap exports GTK paths pointing at its bundled glibc.  Gazebo's
# Qt GUI must not inherit those paths, otherwise libpthread/glibc symbols can
# resolve against /snap/core20 instead of the host libraries.  Ogre2 is also
# more reliable through XWayland/XCB on this host than Qt's Wayland backend.
# Use GZ_GUI_QT_PLATFORM=wayland only when the local graphics stack supports it.
GZ_GUI_QT_PLATFORM ?= xcb
GZ_GUI_VERBOSE ?= 1

# Open live runtime logs in separate xterm windows after `make sim`.  This is
# intentionally optional so the same target can still run from a headless
# shell or CI runner.
SIM_LOG_XTERM ?= 1
SIM_METRICS ?= 1
XTERM ?= xterm
PX4_LOG_XTERM_PID := $(SIM_RUNTIME_DIR)/px4_log_xterm.pid
DDS_LOG_XTERM_PID := $(SIM_RUNTIME_DIR)/dds_log_xterm.pid
ROS_LOG_XTERM_PID := $(SIM_RUNTIME_DIR)/ros_log_xterm.pid

define GZ_GUI_START_COMMAND
source "$(ROS_SETUP)"; \
export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
gui_bin=$$(command -v gz); \
gui_ld="$${LD_LIBRARY_PATH:-}"; \
gui_home="$${HOME:-$(HOME)}"; \
gui_user="$${USER:-$$(id -un)}"; \
gui_runtime="$${XDG_RUNTIME_DIR:-/run/user/$$(id -u)}"; \
gui_display="$${DISPLAY:-:0}"; \
gui_wayland="$(if $(filter xcb,$(GZ_GUI_QT_PLATFORM)),,$${WAYLAND_DISPLAY:-})"; \
gui_session="$(if $(filter xcb,$(GZ_GUI_QT_PLATFORM)),x11,$${XDG_SESSION_TYPE:-wayland})"; \
test -n "$$gui_bin" || { echo "gz command not found"; exit 1; }; \
exec env -i \
HOME="$$gui_home" USER="$$gui_user" LOGNAME="$$gui_user" \
PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
DISPLAY="$$gui_display" WAYLAND_DISPLAY="$$gui_wayland" \
XDG_RUNTIME_DIR="$$gui_runtime" XDG_SESSION_TYPE="$$gui_session" \
XAUTHORITY="$${XAUTHORITY:-}" QT_QPA_PLATFORM="$(GZ_GUI_QT_PLATFORM)" \
LD_LIBRARY_PATH="$$gui_ld" \
ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" \
"$$gui_bin" sim -g -v "$(GZ_GUI_VERBOSE)"
endef

.PHONY: help check check-build check-hil-tools check-hil-firmware-config \
	check-hil-device hil-check hil-config hil-firmware hil-upload jmavsim \
	_jmavsim-start hil hil-stop build px4 dds ros \
	gui sim stop status logs metrics-start metrics-record analyze-tmpc \
	collective-identification-analysis dynamics-identification-analysis \
	mission-start mission-reset mission-execute xterm-logs validation-report

define PX4_START_COMMAND
stale_cache=$$(find "$(PX4_BUILD_DIR)" -type f -name CMakeCache.txt -print 2>/dev/null | while IFS= read -r cache; do \
	source_path=$$(sed -n "s#^CMAKE_HOME_DIRECTORY:INTERNAL=##p" "$$cache"); \
	case "$$source_path" in "$(PX4_DIR)"*) ;; *) printf "%s\\n" "$$cache"; break;; esac; \
done); \
if test -n "$$stale_cache"; then \
	echo "Stale PX4 CMake cache detected: $$stale_cache"; \
	echo "Recreating only the generated PX4 build directory: $(PX4_BUILD_DIR)"; \
	rm -rf "$(PX4_BUILD_DIR)"; \
fi; \
	cd "$(PX4_DIR)" && exec env PX4_SYS_ID="$(PX4_SYS_ID)" PX4_SIM_MODEL_INSTANCE="$(PX4_SIM_MODEL_INSTANCE)" ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)" PX4_UXRCE_DDS_PORT="$(PX4_UXRCE_DDS_PORT)" make "$(PX4_TARGET)" "$(PX4_SIM)" < <(exec tail -f /dev/null)
endef

help:
	@echo "make build   - build the ROS 2 package"
	@echo "make gui     - open Gazebo GUI for an existing SITL server"
	@echo "make sim     - start PX4 SITL + Gazebo GUI + uXRCE-DDS + ROS 2 launch"
	@echo "make hil     - start jMAVSim HITL on the connected PX4 FC (alias: jmavsim)"
	@echo "make hil-check    - validate HITL tools, firmware config and FC serial port"
	@echo "make hil-config   - open the PX4 board configuration UI"
	@echo "make hil-firmware - build the FC firmware with pwm_out_sim"
	@echo "make hil-upload   - explicitly build and flash the HITL firmware"
	@echo "make hil-stop     - stop only jMAVSim started by make hil"
	@echo "make stop    - stop SITL/HIL processes started by this Makefile"
	@echo "make status  - show simulator process status"
	@echo "make logs    - follow PX4, DDS and ROS logs"
	@echo "make sim opens xterm log windows by default; use SIM_LOG_XTERM=0 to disable"
	@echo "make mission-reset - reset the reference to the current hold position"
	@echo "make metrics-record - record TMPC reference/tracking/solve telemetry to CSV"
	@echo "make dynamics-identification-analysis - fit drag/thrust from a speed-sweep metrics CSV"
	@echo "make analyze-tmpc ULOG=... - summarize ULog and optional TMPC CSV"
	@echo "collective ID: ROS_LAUNCH=collective_identification.launch.py SIM_METRICS=0 make sim"
	@echo "collective ID: ros2 service call /collective_identification/start std_srvs/srv/Trigger {}"
	@echo "collective ID: make collective-identification-analysis"
	@echo "make mission-start - trigger mission trajectory execution via ROS 2 service"
	@echo "make mission-execute - wait for PX4 readiness, arm/take off, enter External Mode and run mission"
	@echo "make benchmark     - run side-by-side PID vs MPC log comparison script"
	@echo ""
	@echo "Overrides: PX4_DIR=... HIL_DEVICE=/dev/ttyACM0 HIL_BAUD=921600 HIL_RATE_HZ=250 HIL_HEADLESS=0|1"
	@echo "           PX4_MSGS_SETUP=... DDS_PORT=... GZ_GUI_QT_PLATFORM=wayland|xcb ROS_LAUNCH=... ROS_LAUNCH_ARGS=..."

check-build:
	@test -f "$(ROS_SETUP)" || { echo "Missing ROS setup: $(ROS_SETUP)"; exit 1; }
	@source "$(ROS_SETUP)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	command -v ros2 >/dev/null || { echo "ros2 is not available in PATH"; exit 1; }; \
	command -v colcon >/dev/null || { echo "colcon is not available in PATH"; exit 1; }; \
	ros2 pkg prefix px4_msgs >/dev/null || { echo "px4_msgs is not available; set PX4_MSGS_SETUP"; exit 1; }

check: check-build
	@test -n "$(PX4_DIR)" && test -d "$(PX4_DIR)" || { echo "PX4_DIR is not set or does not exist"; exit 1; }
	@source "$(ROS_SETUP)"; command -v $(DDS_AGENT) >/dev/null || { echo "Missing DDS agent: $(DDS_AGENT)"; exit 1; }
	@source "$(ROS_SETUP)"; command -v gz >/dev/null || { echo "gz is not available in PATH"; exit 1; }
	@command -v make >/dev/null || { echo "make is not available in PATH"; exit 1; }
	@actual_px4=$$(git -C "$(PX4_DIR)" rev-parse HEAD 2>/dev/null || true); \
	if test -n "$(PX4_EXPECTED_COMMIT)" && test "$$actual_px4" != "$(PX4_EXPECTED_COMMIT)"; then \
		echo "PX4 revision mismatch: expected $(PX4_EXPECTED_COMMIT), got $$actual_px4"; exit 1; \
	fi
	@source "$(ROS_SETUP)"; actual_gz=$$(gz sim --version | sed -n 's/.*version //p' | head -n 1); \
	if test -n "$(GZ_EXPECTED_VERSION)" && test "$$actual_gz" != "$(GZ_EXPECTED_VERSION)"; then \
		echo "Gazebo version mismatch: expected $(GZ_EXPECTED_VERSION), got $$actual_gz"; exit 1; \
	fi
	@echo "ROS setup:     $(ROS_SETUP)"
	@echo "PX4 directory: $(PX4_DIR)"
	@echo "PX4 revision:  $$(git -C "$(PX4_DIR)" rev-parse HEAD 2>/dev/null || echo unknown)"
	@echo "Gazebo:        $$(gz sim --version | sed -n 's/.*version //p' | head -n 1)"
	@if test -n "$(PX4_MSGS_SETUP)"; then \
		echo "px4_msgs:      $(PX4_MSGS_SETUP)"; \
	else \
		echo "px4_msgs:      inherited ROS environment"; \
	fi
	@echo "DDS endpoint:  $(DDS_TRANSPORT):$(DDS_PORT)"

check-hil-tools:
	@test -n "$(PX4_DIR)" && test -d "$(PX4_DIR)" || { echo "PX4_DIR is not set or does not exist"; exit 1; }
	@test -x "$(JMAVSIM_RUNNER)" || { echo "Missing jMAVSim runner: $(JMAVSIM_RUNNER)"; exit 1; }
	@test -f "$(HIL_CONFIG)" || { echo "Missing board config: $(HIL_CONFIG)"; exit 1; }
	@command -v make >/dev/null || { echo "make is not available in PATH"; exit 1; }
	@command -v ant >/dev/null || { echo "Apache Ant is missing; install it with: sudo apt install ant"; exit 1; }
	@command -v java >/dev/null || { echo "Java is missing; install a JDK before running jMAVSim"; exit 1; }

check-hil-firmware-config: check-hil-tools
	@grep -qx 'CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y' "$(HIL_CONFIG)" || { \
		echo "pwm_out_sim is not enabled in $(HIL_CONFIG)"; \
		echo "Run 'make hil-config' and enable Modules -> Simulation -> pwm_out_sim."; exit 1; \
	}

check-hil-device: check-hil-tools
	@device="$(HIL_DEVICE)"; \
	if test -z "$$device"; then \
		device=$$(compgen -G '/dev/serial/by-id/*PX4*' | head -n 1); \
	fi; \
	if test -z "$$device"; then device=$$(compgen -G '/dev/ttyACM*' | head -n 1); fi; \
	test -n "$$device" && test -e "$$device" || { echo "PX4 FC serial port not found. Connect and boot the FC, or set HIL_DEVICE=/dev/ttyACM#"; exit 1; }; \
	resolved=$$(readlink -f "$$device"); \
	test -r "$$resolved" && test -w "$$resolved" || { echo "No read/write access to $$resolved; check dialout membership"; exit 1; }; \
	if command -v fuser >/dev/null && fuser "$$resolved" >/dev/null 2>&1; then \
		echo "Serial port $$resolved is busy:"; fuser -v "$$resolved" 2>&1 || true; \
		echo "Close QGroundControl and other serial clients before starting HITL."; exit 1; \
	fi; \
	echo "HIL serial: $$device -> $$resolved"

hil-check: check-hil-firmware-config check-hil-device
	@echo "HIL board:  $(HIL_BOARD)"
	@echo "HIL target: $(HIL_TARGET)"
	@echo "jMAVSim:    baud=$(HIL_BAUD), rate=$(HIL_RATE_HZ) Hz"

hil-config: check-hil-tools
	@cd "$(PX4_DIR)" && $(MAKE) "$(HIL_BOARD)" boardconfig

hil-firmware: check-hil-firmware-config
	@cd "$(PX4_DIR)" && $(MAKE) "$(HIL_TARGET)"

hil-upload: check-hil-device hil-firmware
	@echo "Uploading $(HIL_TARGET) to the connected FC..."
	@cd "$(PX4_DIR)" && $(MAKE) "$(HIL_TARGET)" upload

jmavsim: check-hil-tools check-hil-firmware-config
	@if test -f "$(HIL_PID)" && kill -0 "$$(cat "$(HIL_PID)")" 2>/dev/null; then \
		echo "jMAVSim HITL already running (pid $$(cat "$(HIL_PID)"))."; \
	else \
		rm -f "$(HIL_PID)"; \
		$(MAKE) --no-print-directory _jmavsim-start; \
	fi

_jmavsim-start: hil-check
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@device="$(HIL_DEVICE)"; \
	if test -z "$$device"; then device=$$(compgen -G '/dev/serial/by-id/*PX4*' | head -n 1); fi; \
	if test -z "$$device"; then device=$$(compgen -G '/dev/ttyACM*' | head -n 1); fi; \
	resolved=$$(readlink -f "$$device"); \
	setsid env HEADLESS="$(HIL_HEADLESS)" "$(JMAVSIM_RUNNER)" \
		-q -s -d "$$resolved" -b "$(HIL_BAUD)" -r "$(HIL_RATE_HZ)" \
		>"$(HIL_LOG)" 2>&1 & echo $$! >"$(HIL_PID)"
	@sleep 2; pid=$$(cat "$(HIL_PID)"); \
	if ! kill -0 "$$pid" 2>/dev/null; then \
		echo "jMAVSim exited during startup; inspect $(HIL_LOG)"; \
		tail -n 30 "$(HIL_LOG)"; rm -f "$(HIL_PID)"; exit 1; \
	fi
	@echo "jMAVSim HITL started; log: $(HIL_LOG)"
	@echo "Now open QGroundControl; it should connect through the jMAVSim UDP bridge."

hil: jmavsim

hil-stop:
	@if test -f "$(HIL_PID)"; then \
		pid=$$(cat "$(HIL_PID)"); \
		if kill -0 "$$pid" 2>/dev/null; then \
			echo "Stopping jMAVSim HITL process group $$pid"; \
			kill -TERM -- -"$$pid" 2>/dev/null || kill -TERM "$$pid" 2>/dev/null || true; \
		fi; \
		rm -f "$(HIL_PID)"; \
	else echo "jMAVSim HITL is stopped."; fi

build: check-build
	@avail_mem_kb=$$(awk '/MemAvailable/{print $$2}' /proc/meminfo 2>/dev/null || echo 2000000); \
	swap_free_kb=$$(awk '/SwapFree/{print $$2}' /proc/meminfo 2>/dev/null || echo 0); \
	echo "Build memory preflight: $$((avail_mem_kb/1024)) MB RAM available, $$((swap_free_kb/1024)) MB swap free"; \
	if [ "$$avail_mem_kb" -lt 1200000 ]; then \
		echo "Warning: Low available RAM ($$((avail_mem_kb/1024)) MB). Throttling build to 1 worker to prevent OOM..."; \
		workers=1; \
	else \
		workers=$(ROS_BUILD_PARALLEL_WORKERS); \
	fi; \
	source "$(ROS_SETUP)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	MAKEFLAGS="-j$$workers" \
	CMAKE_BUILD_PARALLEL_LEVEL=$$workers \
	colcon build --base-paths . --packages-select $(ROS_PACKAGE) \
		--parallel-workers $$workers $(ROS_BUILD_ARGS)

px4: check
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@setsid bash -c '$(PX4_START_COMMAND)' \
		>"$(PX4_LOG)" 2>&1 & echo $$! >"$(PX4_PID)"
	@echo "PX4 SITL started; log: $(PX4_LOG)"

dds: check
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@setsid bash -c 'export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; exec "$(DDS_AGENT)" "$(DDS_TRANSPORT)" -p "$(DDS_PORT)"' \
		>"$(DDS_LOG)" 2>&1 & echo $$! >"$(DDS_PID)"
	@echo "uXRCE-DDS agent started; log: $(DDS_LOG)"

ros: check build
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@if test -f "$(ROS_PID)"; then \
		pid=$$(cat "$(ROS_PID)"); \
		if kill -0 "$$pid" 2>/dev/null; then \
			echo "ROS 2 launch already running (pid $$pid)."; exit 1; \
		fi; \
		rm -f "$(ROS_PID)"; \
	fi
	@setsid bash -c 'exec 9>"$(ROS_LOCK)"; if ! flock -n 9; then echo "Another MPC ROS 2 pipeline already holds $(ROS_LOCK)"; exit 75; fi; echo $$$$ >"$(ROS_PID)"; source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; exec ros2 launch "$(ROS_PACKAGE)" "$(ROS_LAUNCH)" $(ROS_LAUNCH_ARGS)' \
		>"$(ROS_LOG)" 2>&1 &
	@sleep 0.2; \
	if ! test -f "$(ROS_PID)"; then \
		echo "ROS 2 launch did not start; inspect $(ROS_LOG)."; exit 1; \
	fi; \
	pid=$$(cat "$(ROS_PID)"); \
	if ! kill -0 "$$pid" 2>/dev/null; then \
		echo "ROS 2 launch exited during startup; inspect $(ROS_LOG)."; \
		rm -f "$(ROS_PID)"; exit 1; \
	fi
	@echo "ROS 2 launch started; log: $(ROS_LOG)"

gui: check
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@if test -f "$(GZ_GUI_PID)"; then \
		pid=$$(cat "$(GZ_GUI_PID)"); \
		if kill -0 "$$pid" 2>/dev/null; then \
			echo "Gazebo GUI already running (pid $$pid)."; exit 0; \
		fi; \
		rm -f "$(GZ_GUI_PID)"; \
	fi
	@if pgrep -u "$$(id -u)" -f '[g]z sim .* -g' >/dev/null 2>&1; then \
		echo "Gazebo GUI already running outside this Makefile."; exit 0; \
	fi
	@setsid bash -c '$(GZ_GUI_START_COMMAND)' \
		>"$(GZ_GUI_LOG)" 2>&1 & echo $$! >"$(GZ_GUI_PID)"
	@echo "Gazebo GUI started; log: $(GZ_GUI_LOG)"

sim: check build
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@if test -f "$(PX4_PID)" || test -f "$(DDS_PID)" || test -f "$(ROS_PID)"; then \
		 echo "A simulator runtime already exists. Run 'make status' or 'make stop' first."; exit 1; \
	fi
	@setsid bash -c '$(PX4_START_COMMAND)' \
		>"$(PX4_LOG)" 2>&1 & echo $$! >"$(PX4_PID)"
	@setsid bash -c 'export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; exec "$(DDS_AGENT)" "$(DDS_TRANSPORT)" -p "$(DDS_PORT)"' \
		>"$(DDS_LOG)" 2>&1 & echo $$! >"$(DDS_PID)"
	deadline=$$((SECONDS + $(SIM_DDS_DISCOVERY_TIMEOUT_SECONDS))); \
	while ! grep -q 'successfully created rt/fmu/out/vehicle_status_v1 data writer' "$(PX4_LOG)" 2>/dev/null || \
		! grep -q 'successfully created rt/fmu/out/arming_check_request_v1 data writer' "$(PX4_LOG)" 2>/dev/null; do \
		if test "$$SECONDS" -ge "$$deadline"; then \
			echo "Timed out waiting for PX4 uXRCE-DDS discovery; inspect $(PX4_LOG) and $(DDS_LOG)"; exit 1; \
		fi; \
		sleep 1; \
	done; \
	sleep 2
	@setsid bash -c 'exec 9>"$(ROS_LOCK)"; if ! flock -n 9; then echo "Another MPC ROS 2 pipeline already holds $(ROS_LOCK)"; exit 75; fi; echo $$$$ >"$(ROS_PID)"; source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; exec ros2 launch "$(ROS_PACKAGE)" "$(ROS_LAUNCH)" $(ROS_LAUNCH_ARGS)' \
		>"$(ROS_LOG)" 2>&1 &
	@if test "$(SIM_METRICS)" = "1"; then $(MAKE) --no-print-directory metrics-start; fi
	@$(MAKE) --no-print-directory xterm-logs
	@echo "Simulation started with Gazebo GUI. No arm/offboard command was sent."
	@echo "Run 'make status' and inspect logs under $(SIM_RUNTIME_DIR)."

stop:
	@set -u; \
	for pid_file in "$(PX4_LOG_XTERM_PID)" "$(DDS_LOG_XTERM_PID)" "$(ROS_LOG_XTERM_PID)" "$(TMPC_METRICS_PID)" "$(HIL_PID)" "$(GZ_GUI_PID)" "$(ROS_PID)" "$(DDS_PID)" "$(PX4_PID)"; do \
		if test -f "$$pid_file"; then \
			pid=$$(cat "$$pid_file"); \
			if kill -0 "$$pid" 2>/dev/null; then \
				echo "Stopping process group $$pid"; kill -TERM -- -"$$pid" 2>/dev/null || kill -TERM "$$pid" 2>/dev/null || true; \
			fi; \
			rm -f "$$pid_file"; \
		fi; \
	done
	@echo "Simulator processes stopped."

status:
	@for entry in "PX4:$(PX4_PID)" "Gazebo GUI:$(GZ_GUI_PID)" "jMAVSim HIL:$(HIL_PID)" "DDS:$(DDS_PID)" "ROS:$(ROS_PID)" "TMPC metrics:$(TMPC_METRICS_PID)" "PX4 log xterm:$(PX4_LOG_XTERM_PID)" "DDS log xterm:$(DDS_LOG_XTERM_PID)" "ROS log xterm:$(ROS_LOG_XTERM_PID)"; do \
		name=$${entry%%:*}; pid_file=$${entry#*:}; \
		if test -f "$$pid_file"; then \
			pid=$$(cat "$$pid_file"); \
			if kill -0 "$$pid" 2>/dev/null; then echo "$$name: running (pid $$pid)"; else echo "$$name: stale pid file ($$pid)"; fi; \
		else echo "$$name: stopped"; fi; \
	done

logs:
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@touch "$(PX4_LOG)" "$(GZ_GUI_LOG)" "$(HIL_LOG)" "$(DDS_LOG)" "$(ROS_LOG)"
	@tail -F "$(PX4_LOG)" "$(GZ_GUI_LOG)" "$(HIL_LOG)" "$(DDS_LOG)" "$(ROS_LOG)"

xterm-logs:
	@if test "$(SIM_LOG_XTERM)" != "1"; then \
		echo "xterm runtime logs disabled (SIM_LOG_XTERM=$(SIM_LOG_XTERM))."; \
		exit 0; \
	fi; \
	if ! command -v "$(XTERM)" >/dev/null 2>&1; then \
		echo "xterm is unavailable; logs remain in $(SIM_RUNTIME_DIR)."; \
		exit 0; \
	fi; \
	if test -z "$${DISPLAY:-}"; then \
		echo "DISPLAY is not set; cannot open xterm log windows."; \
		exit 0; \
	fi; \
	mkdir -p "$(SIM_RUNTIME_DIR)"; \
	start_window() { \
		local title="$$1" log_file="$$2" pid_file="$$3" geometry="$$4"; \
		if test -f "$$pid_file" && kill -0 "$$(cat "$$pid_file")" 2>/dev/null; then \
			echo "$$title log xterm already running."; return; \
		fi; \
		rm -f "$$pid_file"; touch "$$log_file"; \
		setsid "$(XTERM)" -T "$$title" -geometry "$$geometry" \
			-e bash -lc "exec tail -n 100 -F '$$log_file'" >/dev/null 2>&1 & \
		echo $$! > "$$pid_file"; \
		echo "$$title log xterm started."; \
	}; \
	start_window "ROS TMPC log" "$(ROS_LOG)" "$(ROS_LOG_XTERM_PID)" "130x34+40+80"

TMPC_METRICS ?= $(SIM_RUNTIME_DIR)/tpmc_metrics.csv
ULOG ?=
TMPC_ANALYSIS ?= $(SIM_RUNTIME_DIR)/tpmc_analysis.json

metrics-start: check-build
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@if test -f "$(TMPC_METRICS_PID)" && kill -0 "$$(cat "$(TMPC_METRICS_PID)")" 2>/dev/null; then \
		echo "TMPC metrics recorder already running."; \
	else \
		rm -f "$(TMPC_METRICS_PID)"; \
		setsid bash -c 'source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; exec python3 scripts/recording/record_tpmc_metrics.py --output "$(TMPC_METRICS)"' \
			>"$(TMPC_METRICS_LOG)" 2>&1 & echo $$! >"$(TMPC_METRICS_PID)"; \
		echo "TMPC metrics recorder started: $(TMPC_METRICS)"; \
	fi

metrics-record: check-build
	@source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	python3 scripts/recording/record_tpmc_metrics.py --output "$(TMPC_METRICS)"

analyze-tmpc:
	@test -n "$(ULOG)" && test -f "$(ULOG)" || { echo "Set ULOG=/path/to/run.ulg"; exit 1; }
	@args=("$(ULOG)" --output "$(TMPC_ANALYSIS)"); \
	if test -f "$(TMPC_METRICS)"; then args+=(--metrics "$(TMPC_METRICS)"); fi; \
	python3 scripts/analysis/analyze_tpmc_sitl.py "$${args[@]}"

COLLECTIVE_IDENTIFICATION_INPUT ?= $(SIM_RUNTIME_DIR)/collective_identification.csv
COLLECTIVE_IDENTIFICATION_OUTPUT ?= $(SIM_RUNTIME_DIR)/collective_identification.json
DYNAMICS_IDENTIFICATION_INPUT ?= $(TMPC_METRICS)
DYNAMICS_IDENTIFICATION_OUTPUT ?= $(SIM_RUNTIME_DIR)/speed_sweep_identification.json

collective-identification-analysis:
	@test -f "$(COLLECTIVE_IDENTIFICATION_INPUT)" || { echo "Missing $(COLLECTIVE_IDENTIFICATION_INPUT)"; exit 1; }
	@python3 scripts/analysis/analyze_collective_identification.py \
		"$(COLLECTIVE_IDENTIFICATION_INPUT)" --output "$(COLLECTIVE_IDENTIFICATION_OUTPUT)"

dynamics-identification-analysis:
	@test -f "$(DYNAMICS_IDENTIFICATION_INPUT)" || { echo "Missing $(DYNAMICS_IDENTIFICATION_INPUT)"; exit 1; }
	@python3 scripts/analysis/analyze_speed_sweep_identification.py \
		"$(DYNAMICS_IDENTIFICATION_INPUT)" --output "$(DYNAMICS_IDENTIFICATION_OUTPUT)"

PID_LOG ?=
MPC_LOG ?=
MISSION_JSON ?= config/missions/benchmark_square.json
BENCHMARK_OUT ?= mission_benchmark_comparison.png

mission-start:
	@source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	ros2 service call /reference_generator_node/start_mission std_srvs/srv/Trigger {}

mission-reset:
	@source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	ros2 service call /reference_generator_node/reset_mission std_srvs/srv/Trigger {}

MISSION_READY_TIMEOUT_SECONDS ?= 120
MISSION_EXTERNAL_TIMEOUT_SECONDS ?= 60
MISSION_EXECUTION_TIMEOUT_SECONDS ?= 360

mission-execute:
	@test -f "$(PX4_PID)" && kill -0 "$$(cat "$(PX4_PID)")" 2>/dev/null || { echo "PX4 SITL is not running; start it with make sim."; exit 1; }
	@deadline=$$((SECONDS + $(MISSION_READY_TIMEOUT_SECONDS))); \
	while ! grep -q 'Ready for takeoff' "$(PX4_LOG)" 2>/dev/null; do \
		if test "$$SECONDS" -ge "$$deadline"; then \
			echo "PX4 did not become ready for takeoff; refusing to force-arm."; \
			tail -n 30 "$(PX4_LOG)"; exit 1; \
		fi; \
		sleep 1; \
	done; \
	source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	ros2 service call /mpc_mode_executor/start std_srvs/srv/Trigger {}; \
	deadline=$$((SECONDS + $(MISSION_EXTERNAL_TIMEOUT_SECONDS))); \
	while ! grep -q 'External Mode selected by PX4' "$(ROS_LOG)" 2>/dev/null; do \
		if grep -q 'Sequence failed during' "$(ROS_LOG)" 2>/dev/null || test "$$SECONDS" -ge "$$deadline"; then \
			echo "External Mode entry failed; inspect $(ROS_LOG)."; exit 1; \
		fi; \
		sleep 1; \
	done; \
	ros2 service call /reference_generator_node/start_mission std_srvs/srv/Trigger {}; \
	deadline=$$((SECONDS + $(MISSION_EXECUTION_TIMEOUT_SECONDS))); \
	while ! grep -q 'Autonomous sequence complete' "$(ROS_LOG)" 2>/dev/null; do \
		if grep -q 'Sequence failed during mission wait' "$(ROS_LOG)" 2>/dev/null || test "$$SECONDS" -ge "$$deadline"; then \
			echo "Mission execution did not complete; request landing before stopping SITL."; exit 1; \
		fi; \
		sleep 2; \
	done; \
	echo "Mission and native landing completed successfully."

arm:
	@source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{command: 400, param1: 1.0, param2: 21196.0, target_system: $(PX4_SYS_ID), target_component: 1, source_system: $(PX4_SYS_ID), source_component: 1, from_external: true}"

disarm:
	@source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{command: 400, param1: 0.0, param2: 21196.0, target_system: $(PX4_SYS_ID), target_component: 1, source_system: $(PX4_SYS_ID), source_component: 1, from_external: true}"

offboard:
	@source "$(ROS_SETUP)"; export ROS_DOMAIN_ID="$(ROS_DOMAIN_ID)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{command: 176, param1: 1.0, param2: 6.0, target_system: $(PX4_SYS_ID), target_component: 1, source_system: $(PX4_SYS_ID), source_component: 1, from_external: true}"

mission-run: mission-start
	@sleep 1.0
	@$(MAKE) --no-print-directory arm
	@sleep 0.5
	@$(MAKE) --no-print-directory offboard
	@echo "Mission initiated: trajectory streaming, vehicle armed, and offboard control active."

pid-mission-run:
	@python3 scripts/execution/run_px4_pid_mission.py --mission "$(MISSION_JSON)"

pid-plan-gen:
	@python3 scripts/execution/run_px4_pid_mission.py --mission "$(MISSION_JSON)" --plan-only

benchmark:
	@test -n "$(PID_LOG)" && test -f "$(PID_LOG)" || { echo "PID_LOG not found. Usage: make benchmark PID_LOG=path/to/pid.ulg MPC_LOG=path/to/mpc.ulg"; exit 1; }
	@test -n "$(MPC_LOG)" && test -f "$(MPC_LOG)" || { echo "MPC_LOG not found. Usage: make benchmark PID_LOG=path/to/pid.ulg MPC_LOG=path/to/mpc.ulg"; exit 1; }
	@python3 scripts/analysis/compare_mission_logs.py "$(PID_LOG)" "$(MPC_LOG)" --mission "$(MISSION_JSON)" --out "$(BENCHMARK_OUT)"

VALIDATION_REPORT ?= $(SIM_RUNTIME_DIR)/validation_report.json

validation-report:
	@test -f "$(TMPC_METRICS)" || { echo "Missing $(TMPC_METRICS); run a SITL validation flight first."; exit 1; }
	@test -n "$(MISSION_JSON)" && test -f "$(MISSION_JSON)" || { echo "MISSION_JSON must name the validated mission JSON."; exit 1; }
	@python3 scripts/analysis/analyze_validation_run.py \
		"$(TMPC_METRICS)" --mission "$(MISSION_JSON)" --output "$(VALIDATION_REPORT)"
