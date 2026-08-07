# Third-party license audit

This is an inventory for engineering traceability, not legal advice. The
current project scope is private, internal development with no redistribution.
If that scope changes, every dependency must retain its upstream copyright and
license terms before release.

| Component | Version/revision | Declared/observed license | Evidence | Audit status |
|---|---|---|---|---|
| PX4 Autopilot | official `v1.17.0` commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | BSD 3-Clause | PX4 `LICENSE` | Reviewed |
| `px4_msgs` | `86d8239e962f6939e05c3737784f60c02fa884db` (`v1.17.0`) | BSD 3-Clause | `px4_msgs/LICENSE` and package metadata | Reviewed; message compatibility verified |
| `mrs_mpc_solvers` | `f173ea285cce7bbbfc1dbc382e31c761f44b0963` | BSD 3-Clause declared | [`MRS_MPC_SOLVERS_LICENSE_EVIDENCE.md`](MRS_MPC_SOLVERS_LICENSE_EVIDENCE.md), upstream `package.xml`; no `LICENSE`, `COPYING` or `NOTICE` in the revision | Accepted for private development; redistribution blocked |
| Eigen3 | Debian `3.4.0-4build0.1` | MPL-2.0 with documented LGPL/BSD component exceptions | `/usr/share/doc/libeigen3-dev/copyright` | Reviewed for current host |
| GoogleTest | Debian `1.14.0-1` | BSD 3-Clause | `/usr/share/doc/libgtest-dev/copyright` | Reviewed for current host |
| ROS 2 `ament_cmake`/`rclcpp` | Jazzy host packages | Apache License 2.0 declared by package metadata | `/opt/ros/jazzy/share/*/package.xml` | Partial; full transitive inventory pending |
| Gazebo Sim | Host packages `8.14.0-1~noble` | Apache License 2.0 | `/usr/share/doc/gz-sim8-cli/copyright`, `/usr/share/doc/libgz-sim8/copyright` | Reviewed for selected direct packages |

## Required actions before redistribution

- Obtain the upstream BSD-3-Clause license text and copyright notice for
  `mrs_mpc_solvers`; see [`MRS_MPC_SOLVERS_LICENSE_EVIDENCE.md`](MRS_MPC_SOLVERS_LICENSE_EVIDENCE.md).
- Generate a complete license inventory for the selected ROS/Gazebo package
  closure, not just the direct packages inspected here.
- Store the exact license evidence with the dependency revision and package
  manifest used by CI.
