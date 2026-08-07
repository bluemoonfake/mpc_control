#!/usr/bin/env bash

# Source this file before building/running PX4 SITL with system Gazebo
# Harmonic. It removes ROS Jazzy's bundled Gazebo 8.11 paths and inherited
# Snap runtime libraries from the PX4 process while keeping the rest of the
# ROS 2 environment available.
#
# Usage:
#   source tools/px4_gazebo_harmonic_env.sh
#   gz sim --version
#   make px4_sitl gz_x500

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "source this file; do not execute it" >&2
  exit 2
fi

_mpc_filter_ros_gz_paths() {
  local value="${1:-}"
  local result=""
  local item
  local separator=""

  IFS=':' read -r -a items <<< "$value"
  for item in "${items[@]}"; do
    [[ -z "$item" ]] && continue
    if [[ "$item" == /opt/ros/jazzy/opt/*_vendor* ||
          "$item" == /snap/* ||
          "$item" == /var/lib/snapd/lib/* ]]; then
      continue
    fi
    result="${result}${separator}${item}"
    separator=":"
  done
  printf '%s' "$result"
}

export GZ_CONFIG_PATH=/usr/share/gz
export LD_LIBRARY_PATH="$(_mpc_filter_ros_gz_paths "${LD_LIBRARY_PATH:-}")"
export CMAKE_PREFIX_PATH="$(_mpc_filter_ros_gz_paths "${CMAKE_PREFIX_PATH:-}")"
# A Snap-injected preload can bind a different glibc than the system Gazebo
# packages. The system runtime must resolve against the host libraries.
unset LD_PRELOAD
unset SNAP_LIBRARY_PATH

# VS Code installed as a Snap exports GTK/GIO module paths from the Snap
# runtime. Gazebo's Qt GUI can load those GTK modules indirectly, which mixes
# the Snap core20 glibc with the host glibc and causes a libpthread lookup
# failure. Leave the host desktop/session variables intact, but remove the
# Snap-specific GTK/GIO module selectors.
unset GTK_PATH
unset GTK_EXE_PREFIX
unset GTK_MODULES
unset GDK_PIXBUF_MODULEDIR
unset GDK_PIXBUF_MODULE_FILE
unset GIO_MODULE_DIR
unset GTK_IM_MODULE_FILE

unset -f _mpc_filter_ros_gz_paths
