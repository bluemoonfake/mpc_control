#include "mpc_controller/mission/mission_json_parser.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

std::string fixturePath(const std::string &file_name) {
  return std::string(MPC_TEST_FIXTURE_DIR) + "/missions/" + file_name;
}

TEST(MissionJsonParser, LoadsMissionDefaultsAndItems) {
  mpc_controller::mission::MissionJsonParser parser;

  const auto mission = parser.load(fixturePath("minimal_mission.json"));

  ASSERT_TRUE(mission.valid) << mission.error;
  EXPECT_DOUBLE_EQ(mission.defaults.horizontal_velocity_m_s, 3.0);
  EXPECT_DOUBLE_EQ(mission.defaults.maximum_acceleration_m_s2, 2.0);
  EXPECT_DOUBLE_EQ(mission.defaults.maximum_jerk_m_s3, 4.0);
  ASSERT_EQ(mission.items.size(), 3U);
  EXPECT_EQ(mission.items[0].type, mpc_controller::mission::ItemType::Takeoff);
  EXPECT_EQ(mission.items[1].type, mpc_controller::mission::ItemType::Waypoint);
  EXPECT_DOUBLE_EQ(mission.items[1].waypoint.position_enu[0], 5.0);
  EXPECT_EQ(mission.items[2].type, mpc_controller::mission::ItemType::Land);
}

TEST(MissionJsonParser, ReportsMalformedJsonWithoutThrowing) {
  mpc_controller::mission::MissionJsonParser parser;

  const auto mission = parser.load(fixturePath("malformed_mission.json"));

  EXPECT_FALSE(mission.valid);
  EXPECT_FALSE(mission.error.empty());
}

TEST(MissionJsonParser, RejectsInvalidMissionLimitsBeforeRuntimeLoading) {
  mpc_controller::mission::MissionJsonParser parser;

  EXPECT_FALSE(parser.load(fixturePath("invalid_heading_rate.json")).valid);
  EXPECT_FALSE(parser.load(fixturePath("invalid_tpmc.json")).valid);
}

TEST(MissionJsonParser, ParsesRuntimeReturnToStartItem) {
  mpc_controller::mission::MissionJsonParser parser;

  const auto mission = parser.load(fixturePath("rtl_mission.json"));

  ASSERT_TRUE(mission.valid) << mission.error;
  ASSERT_EQ(mission.items.size(), 3U);
  EXPECT_EQ(mission.items.back().type, mpc_controller::mission::ItemType::Rtl);
}

} // namespace
