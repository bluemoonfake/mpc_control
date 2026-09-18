#include "mpc_controller/mission/mission_trajectory.hpp"

#include <gtest/gtest.h>

namespace {

mpc_controller::mission::Mission makeMission() {
  mpc_controller::mission::Mission mission;
  mission.valid = true;
  mission.defaults.horizontal_velocity_m_s = 2.0;
  mission.defaults.vertical_velocity_m_s = 1.0;

  mpc_controller::mission::MissionItem takeoff;
  takeoff.type = mpc_controller::mission::ItemType::Takeoff;
  takeoff.id = "takeoff";
  takeoff.waypoint.position_enu[2] = 5.0;
  mission.items.push_back(takeoff);

  mpc_controller::mission::MissionItem waypoint;
  waypoint.type = mpc_controller::mission::ItemType::Waypoint;
  waypoint.id = "wp1";
  waypoint.waypoint.position_enu = {4.0, 0.0, 5.0};
  mission.items.push_back(waypoint);
  return mission;
}

mpc_controller::mission::Mission makeTwoWaypointMission() {
  auto mission = makeMission();
  mpc_controller::mission::MissionItem waypoint;
  waypoint.type = mpc_controller::mission::ItemType::Waypoint;
  waypoint.id = "wp2";
  // The successor intentionally reverses lateral direction.  A horizon that
  // crosses the unaccepted wp1 would otherwise command this negative Y leg.
  waypoint.waypoint.position_enu = {8.0, -6.0, 5.0};
  mission.items.push_back(waypoint);
  return mission;
}

mpc_controller::mission::Mission makeReturnMission() {
  auto mission = makeMission();
  mpc_controller::mission::MissionItem rtl;
  rtl.type = mpc_controller::mission::ItemType::Rtl;
  rtl.id = "return_home";
  mission.items.push_back(rtl);
  return mission;
}

TEST(MissionReferenceGenerator, ProducesDeterministicRollingHorizonWithoutRos) {
  mpc_controller::mission::MissionReferenceGenerator::Config config;
  config.reference.hold_position = {0.0, 0.0, 1.0};
  config.horizon_seconds = 3.0;
  config.sample_period_seconds = 0.1;
  mpc_controller::mission::MissionReferenceGenerator generator(config);

  std::string error;
  ASSERT_TRUE(generator.setMission(makeMission(), error)) << error;
  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {0.0, 0.0, 1.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(10.0));

  const auto first = generator.update(12.0);
  const auto repeated = generator.update(12.0);
  ASSERT_EQ(first.samples.size(), 31U);
  ASSERT_EQ(first.samples.size(), repeated.samples.size());
  EXPECT_DOUBLE_EQ(first.samples.front().position[2],
                   repeated.samples.front().position[2]);
  EXPECT_NEAR(first.samples.front().position[2], 3.0, 1e-12);
  EXPECT_GT(first.samples.front().velocity[2], 0.0);
}

TEST(MissionReferenceGenerator, AdvancesUsingMeasuredVehicleState) {
  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  ASSERT_TRUE(generator.setMission(makeMission(), error)) << error;

  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {0.0, 0.0, 1.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(0.0));

  state.position = {0.0, 0.0, 5.0};
  generator.updateVehicleState(state);
  const auto update = generator.update(4.0);
  EXPECT_TRUE(update.waypoint_reached);
  EXPECT_FALSE(update.mission_completed);
  EXPECT_EQ(update.reached_waypoint_index, 0U);
  EXPECT_EQ(generator.currentWaypointIndex(), 1U);
}

TEST(MissionReferenceGenerator, ChangeSettingsApplyOnlyToFollowingWaypoints) {
  auto mission = makeMission();
  mission.defaults.maximum_acceleration_m_s2 = 2.0;
  mission.defaults.maximum_jerk_m_s3 = 4.0;
  mpc_controller::mission::MissionItem change;
  change.type = mpc_controller::mission::ItemType::ChangeSettings;
  change.settings.horizontal_velocity_m_s = 3.0;
  change.settings.maximum_acceleration_m_s2 = 1.0;
  change.settings.maximum_jerk_m_s3 = 2.0;
  mission.items.push_back(change);
  auto second = mission.items[1];
  second.id = "wp2";
  mission.items.push_back(second);
  mpc_controller::mission::MissionItem reset;
  reset.type = mpc_controller::mission::ItemType::ChangeSettings;
  reset.settings.reset_all = true;
  mission.items.push_back(reset);
  auto third = mission.items[1];
  third.id = "wp3";
  mission.items.push_back(third);

  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  ASSERT_TRUE(generator.setMission(mission, error)) << error;
  const auto &waypoints = generator.waypoints();
  ASSERT_EQ(waypoints.size(), 4U);
  EXPECT_DOUBLE_EQ(waypoints[1].horizontal_speed, 2.0);
  EXPECT_DOUBLE_EQ(waypoints[2].horizontal_speed, 3.0);
  EXPECT_DOUBLE_EQ(waypoints[2].maximum_acceleration_m_s2, 1.0);
  EXPECT_DOUBLE_EQ(waypoints[2].maximum_jerk_m_s3, 2.0);
  EXPECT_DOUBLE_EQ(waypoints[3].horizontal_speed, 2.0);
  EXPECT_DOUBLE_EQ(waypoints[3].maximum_acceleration_m_s2, 2.0);
  EXPECT_DOUBLE_EQ(waypoints[3].maximum_jerk_m_s3, 4.0);
}

TEST(MissionReferenceGenerator, DoesNotAdvanceOnElapsedTimeOutsideAcceptance) {
  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  ASSERT_TRUE(generator.setMission(makeMission(), error)) << error;

  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {0.0, 0.0, 1.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(0.0));

  // The old timeout fallback accepted up to acceptance_radius * 2.5, which
  // would incorrectly advance this takeoff at the 6.22 m miss from SITL.
  state.position = {0.0, 0.0, -1.22};
  generator.updateVehicleState(state);
  const auto update = generator.update(4.0);

  EXPECT_FALSE(update.waypoint_reached);
  EXPECT_NEAR(update.distance_to_waypoint_m, 6.22, 1e-12);
  EXPECT_EQ(generator.currentWaypointIndex(), 0U);
}

TEST(MissionReferenceGenerator,
     DoesNotAdvanceWhenCrossingPlaneFarFromWaypoint) {
  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  ASSERT_TRUE(generator.setMission(makeMission(), error)) << error;

  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {0.0, 0.0, 1.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(0.0));

  // Enter the first waypoint legitimately.
  state.position = {0.0, 0.0, 5.0};
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.update(4.0).waypoint_reached);
  ASSERT_EQ(generator.currentWaypointIndex(), 1U);

  // This sample is beyond wp1's finish plane but 5 m laterally away.  It
  // must remain on wp1 instead of cutting directly to the next segment.
  state.position = {5.0, 5.0, 5.0};
  generator.updateVehicleState(state);
  const auto update = generator.update(6.0);

  EXPECT_TRUE(update.crossed_finish_plane);
  EXPECT_GT(update.distance_to_waypoint_m, 2.5);
  EXPECT_FALSE(update.waypoint_reached);
  EXPECT_EQ(generator.currentWaypointIndex(), 1U);
}

TEST(MissionReferenceGenerator, DoesNotPreviewAnUnacceptedSuccessorWaypoint) {
  mpc_controller::mission::MissionReferenceGenerator::Config config;
  config.reference.hold_position = {0.0, 0.0, 1.0};
  config.horizon_seconds = 5.0;
  config.sample_period_seconds = 1.0;
  mpc_controller::mission::MissionReferenceGenerator generator(config);
  std::string error;
  ASSERT_TRUE(generator.setMission(makeTwoWaypointMission(), error)) << error;

  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {0.0, 0.0, 1.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(0.0));

  // Enter wp1 legitimately.  Keep the measured vehicle far from wp1 so it
  // cannot advance to wp2 even after wp1's nominal two-second leg duration.
  state.position = {0.0, 0.0, 5.0};
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.update(4.0).waypoint_reached);
  ASSERT_EQ(generator.currentWaypointIndex(), 1U);

  state.position = {0.0, 0.0, 5.0};
  generator.updateVehicleState(state);
  const auto update = generator.update(7.0);

  ASSERT_FALSE(update.waypoint_reached);
  ASSERT_EQ(generator.currentWaypointIndex(), 1U);
  ASSERT_FALSE(update.samples.empty());
  for (const auto &sample : update.samples) {
    EXPECT_DOUBLE_EQ(sample.position[0], 4.0);
    EXPECT_DOUBLE_EQ(sample.position[1], 0.0);
    EXPECT_DOUBLE_EQ(sample.position[2], 5.0);
    EXPECT_DOUBLE_EQ(sample.velocity[0], 0.0);
    EXPECT_DOUBLE_EQ(sample.velocity[1], 0.0);
    EXPECT_DOUBLE_EQ(sample.velocity[2], 0.0);
  }
}

TEST(MissionReferenceGenerator, RtlReturnsToMissionStartAtMissionAltitude) {
  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  ASSERT_TRUE(generator.setMission(makeReturnMission(), error)) << error;

  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {2.0, -3.0, 2.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(0.0));

  ASSERT_EQ(generator.waypoints().back().id, "return_home");
  EXPECT_EQ(generator.waypoints().back().position,
            (std::array<double, 3>{2.0, -3.0, 5.0}));

  state.position = {40.0, 50.0, 5.0};
  generator.updateVehicleState(state);
  EXPECT_EQ(generator.waypoints().back().position,
            (std::array<double, 3>{2.0, -3.0, 5.0}));
}

TEST(MissionReferenceGenerator, RtlRebindsWhenMissionIsRestarted) {
  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  ASSERT_TRUE(generator.setMission(makeReturnMission(), error)) << error;

  mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
  state.position = {1.0, 2.0, 5.0};
  state.valid = true;
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(0.0));

  state.position = {-4.0, 6.0, 5.0};
  generator.updateVehicleState(state);
  ASSERT_TRUE(generator.start(10.0));
  EXPECT_EQ(generator.waypoints().back().position,
            (std::array<double, 3>{-4.0, 6.0, 5.0}));
}

TEST(MissionReferenceGenerator, RejectsNonTerminalRtl) {
  auto mission = makeReturnMission();
  mpc_controller::mission::MissionItem waypoint;
  waypoint.type = mpc_controller::mission::ItemType::Waypoint;
  waypoint.id = "after_rtl";
  waypoint.waypoint.position_enu = {10.0, 0.0, 5.0};
  mission.items.push_back(waypoint);

  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  EXPECT_FALSE(generator.setMission(mission, error));
  EXPECT_EQ(error, "rtl must be the final mission item");
  EXPECT_TRUE(generator.waypoints().empty());
}

TEST(MissionReferenceGenerator, RejectsRtlWithoutPrecedingWaypoint) {
  mpc_controller::mission::Mission mission;
  mission.valid = true;
  mpc_controller::mission::MissionItem rtl;
  rtl.type = mpc_controller::mission::ItemType::Rtl;
  mission.items.push_back(rtl);

  mpc_controller::mission::MissionReferenceGenerator generator({});
  std::string error;
  EXPECT_FALSE(generator.setMission(mission, error));
  EXPECT_EQ(error, "rtl requires a preceding executable waypoint");
  EXPECT_TRUE(generator.waypoints().empty());
}

} // namespace
