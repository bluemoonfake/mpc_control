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

TEST(MissionReferenceGenerator, DoesNotAdvanceWhenCrossingPlaneFarFromWaypoint) {
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

} // namespace
