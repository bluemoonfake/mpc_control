# Compact simulator orchestration for the M5 normalized-wrench path.
#
# The Makefile does not arm PX4 and does not change PX4 failsafe parameters.
# PX4 and px4_msgs remain external dependencies of this source-only branch.

SHELL := /bin/bash

ROS_SETUP ?= /opt/ros/jazzy/setup.bash
PX4_DIR ?= $(firstword $(wildcard $(CURDIR)/third_party/PX4-Autopilot /tmp/mpc_controller_PX4-Autopilot_backup $(HOME)/PX4_17/PX4-Autopilot $(HOME)/Dev/PX4_tracker/PX4-Autopilot))
PX4_MSGS_SETUP ?= $(firstword $(wildcard $(CURDIR)/install/px4_msgs/local_setup.bash $(CURDIR)/install/ros_px4_msgs/local_setup.bash /tmp/mpc_controller_px4_msgs_install.*/local_setup.bash $(HOME)/Dev/precision-land/install/local_setup.bash))
ROS_WORKSPACE_SETUP ?= $(CURDIR)/install/local_setup.bash
PX4_TARGET ?= px4_sitl
PX4_SIM ?= gz_x500
PX4_BUILD_DIR ?= $(PX4_DIR)/build/px4_sitl_default
PX4_EXPECTED_COMMIT ?= 0b6e4687defb353a34201951809efd3f0040a9ba
GZ_EXPECTED_VERSION ?= 8.11.0

DDS_AGENT ?= MicroXRCEAgent
DDS_TRANSPORT ?= udp4
DDS_PORT ?= 8888

ROS_PACKAGE ?= mpc_controller
ROS_LAUNCH ?= m5_sitl.launch.py
ROS_LAUNCH_ARGS ?=
ROS_BUILD_ARGS ?= --symlink-install

SIM_RUNTIME_DIR ?= /tmp/mpc_controller_sim
PX4_LOG := $(SIM_RUNTIME_DIR)/px4.log
DDS_LOG := $(SIM_RUNTIME_DIR)/dds.log
ROS_LOG := $(SIM_RUNTIME_DIR)/ros.log
GZ_GUI_LOG := $(SIM_RUNTIME_DIR)/gazebo_gui.log
PX4_PID := $(SIM_RUNTIME_DIR)/px4.pid
DDS_PID := $(SIM_RUNTIME_DIR)/dds.pid
ROS_PID := $(SIM_RUNTIME_DIR)/ros.pid
GZ_GUI_PID := $(SIM_RUNTIME_DIR)/gazebo_gui.pid

# The VS Code Snap exports GTK paths pointing at its bundled glibc.  Gazebo's
# Qt GUI must not inherit those paths, otherwise libpthread/glibc symbols can
# resolve against /snap/core20 instead of the host libraries.  Ogre2 is also
# more reliable through XWayland/XCB on this host than Qt's Wayland backend.
# Use GZ_GUI_QT_PLATFORM=wayland only when the local graphics stack supports it.
GZ_GUI_QT_PLATFORM ?= xcb
GZ_GUI_VERBOSE ?= 1

define GZ_GUI_START_COMMAND
source "$(ROS_SETUP)"; \
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
"$$gui_bin" sim -g -v "$(GZ_GUI_VERBOSE)"
endef

.PHONY: help check check-build build px4 dds ros gui sim stop status logs

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
cd "$(PX4_DIR)" && exec make "$(PX4_TARGET)" "$(PX4_SIM)"
endef

help:
	@echo "make build   - build the ROS 2 package"
	@echo "make gui     - open Gazebo GUI for an existing SITL server"
	@echo "make sim     - start PX4 SITL + Gazebo GUI + uXRCE-DDS + ROS 2 launch"
	@echo "make stop    - stop only processes started by make sim"
	@echo "make status  - show simulator process status"
	@echo "make logs    - follow PX4, DDS and ROS logs"
	@echo ""
	@echo "Overrides: PX4_DIR=... PX4_MSGS_SETUP=... DDS_PORT=... GZ_GUI_QT_PLATFORM=wayland|xcb ROS_LAUNCH_ARGS=..."

check-build:
	@test -f "$(ROS_SETUP)" || { echo "Missing ROS setup: $(ROS_SETUP)"; exit 1; }
	@source "$(ROS_SETUP)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
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

build: check-build
	@source "$(ROS_SETUP)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	colcon build --base-paths . --packages-select $(ROS_PACKAGE) $(ROS_BUILD_ARGS)

px4: check
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@setsid bash -c '$(PX4_START_COMMAND)' \
		>"$(PX4_LOG)" 2>&1 & echo $$! >"$(PX4_PID)"
	@echo "PX4 SITL started; log: $(PX4_LOG)"

dds: check
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@setsid bash -c 'exec "$(DDS_AGENT)" "$(DDS_TRANSPORT)" -p "$(DDS_PORT)"' \
		>"$(DDS_LOG)" 2>&1 & echo $$! >"$(DDS_PID)"
	@echo "uXRCE-DDS agent started; log: $(DDS_LOG)"

ros: check build
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@source "$(ROS_SETUP)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	setsid bash -c 'source "$(ROS_SETUP)"; if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; exec ros2 launch "$(ROS_PACKAGE)" "$(ROS_LAUNCH)" $(ROS_LAUNCH_ARGS)' \
		>"$(ROS_LOG)" 2>&1 & echo $$! >"$(ROS_PID)"
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
	@setsid bash -c 'exec "$(DDS_AGENT)" "$(DDS_TRANSPORT)" -p "$(DDS_PORT)"' \
		>"$(DDS_LOG)" 2>&1 & echo $$! >"$(DDS_PID)"
	@sleep 2
	@$(MAKE) --no-print-directory gui
	@source "$(ROS_SETUP)"; \
	if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; \
	if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; \
	setsid bash -c 'source "$(ROS_SETUP)"; if test -n "$(PX4_MSGS_SETUP)" && test -f "$(PX4_MSGS_SETUP)"; then source "$(PX4_MSGS_SETUP)"; fi; if test -f "$(ROS_WORKSPACE_SETUP)"; then source "$(ROS_WORKSPACE_SETUP)"; fi; exec ros2 launch "$(ROS_PACKAGE)" "$(ROS_LAUNCH)" $(ROS_LAUNCH_ARGS)' \
		>"$(ROS_LOG)" 2>&1 & echo $$! >"$(ROS_PID)"
	@echo "Simulation started with Gazebo GUI. No arm/offboard command was sent."
	@echo "Run 'make status' and inspect logs under $(SIM_RUNTIME_DIR)."

stop:
	@set -u; \
	for pid_file in "$(GZ_GUI_PID)" "$(ROS_PID)" "$(DDS_PID)" "$(PX4_PID)"; do \
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
	@for entry in "PX4:$(PX4_PID)" "Gazebo GUI:$(GZ_GUI_PID)" "DDS:$(DDS_PID)" "ROS:$(ROS_PID)"; do \
		name=$${entry%%:*}; pid_file=$${entry#*:}; \
		if test -f "$$pid_file"; then \
			pid=$$(cat "$$pid_file"); \
			if kill -0 "$$pid" 2>/dev/null; then echo "$$name: running (pid $$pid)"; else echo "$$name: stale pid file ($$pid)"; fi; \
		else echo "$$name: stopped"; fi; \
	done

logs:
	@mkdir -p "$(SIM_RUNTIME_DIR)"
	@tail -F "$(PX4_LOG)" "$(GZ_GUI_LOG)" "$(DDS_LOG)" "$(ROS_LOG)"
