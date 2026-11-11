// Copyright 2026 Instinct Robotics
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "comau_hardware_interface/joint_mapping.hpp"

using comau_hardware_interface::JointMapping;

namespace
{
// The hard-coded mapping the driver had before it became a table: kept here as the reference the default must equal.
constexpr float kJ6ZeroDeg = 34.5f;

double legacy_to_ros(size_t index, float deg)
{
  switch (index) {
    case 0: return static_cast<double>(-deg) * M_PI / 180.0;
    case 2: return static_cast<double>(-90.0f - deg) * M_PI / 180.0;
    case 3: return static_cast<double>(-deg) * M_PI / 180.0;
    case 5: return static_cast<double>(kJ6ZeroDeg - deg) * M_PI / 180.0;
    default: return static_cast<double>(deg) * M_PI / 180.0;
  }
}

float legacy_to_comau(size_t index, double rad)
{
  float deg = static_cast<float>(rad * 180.0 / M_PI);
  switch (index) {
    case 0: return -deg;
    case 2: return -90.0f - deg;
    case 3: return -deg;
    case 5: return kJ6ZeroDeg - deg;
    default: return deg;
  }
}
}  // namespace

TEST(JointMapping, DefaultEqualsTheFormerHardCodedMappingForEveryJoint)
{
  const auto m = JointMapping::racer_default();
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> deg(-360.0f, 360.0f);
  std::uniform_real_distribution<double> rad(-6.3, 6.3);
  for (size_t i = 0; i < 8; ++i) {  // joints 6 and 7 are beyond the table: unchanged
    for (int n = 0; n < 2000; ++n) {
      const float d = deg(rng);
      const double r = rad(rng);
      EXPECT_EQ(m.to_model(i, d), legacy_to_ros(i, d)) << "joint " << i;
      EXPECT_EQ(m.to_controller(i, r), legacy_to_comau(i, r)) << "joint " << i;
    }
  }
}

TEST(JointMapping, RoundTripIsTheIdentityForAnyTable)
{
  JointMapping m = JointMapping::racer_default();
  m.override_with({1, -1, 1, 1}, {10.0f, -20.0f, 30.5f, 0.0f}, 4);
  for (size_t i = 0; i < 4; ++i) {
    for (float d : {-170.0f, -12.5f, 0.0f, 33.3f, 179.0f}) {
      const double rad = m.to_model(i, d);
      EXPECT_NEAR(m.to_controller(i, rad), d, 1e-3) << "joint " << i;
    }
  }
}

TEST(JointMapping, OverrideAppliesPerJointSignAndOffset)
{
  JointMapping m = JointMapping::racer_default();
  m.override_with({-1, 1, -1, -1}, {0.0f, 0.0f, -85.0f, 0.0f}, 4);
  EXPECT_NEAR(m.to_model(2, 10.0f), (-1.0 * 10.0 - 85.0) * M_PI / 180.0, 1e-6);
  EXPECT_NEAR(m.to_model(1, 10.0f), 10.0 * M_PI / 180.0, 1e-6);
  EXPECT_NEAR(m.to_controller(2, (-1.0 * 10.0 - 85.0) * M_PI / 180.0), 10.0f, 1e-3);
}

TEST(JointMapping, OneListKeepsTheDefaultsOfTheOtherForTheFirstJoints)
{
  JointMapping m = JointMapping::racer_default();
  m.override_with({}, {5.0f, 0.0f, 0.0f, 0.0f}, 4);  // offsets only
  EXPECT_FLOAT_EQ(m.sign(0), -1.0f);
  EXPECT_FLOAT_EQ(m.sign(2), -1.0f);
  EXPECT_FLOAT_EQ(m.offset(0), 5.0f);
  EXPECT_FLOAT_EQ(m.offset(2), 0.0f);
}

TEST(JointMapping, FewerJointsUseTheFirstEntriesOfTheDefaultTable)
{
  JointMapping m = JointMapping::racer_default();
  m.override_with({1, 1, 1, 1}, {}, 4);  // only signs: offsets keep the built-in values of joints 0..3
  EXPECT_FLOAT_EQ(m.offset(2), -90.0f);
  EXPECT_FLOAT_EQ(m.sign(3), 1.0f);
}

TEST(JointMapping, RejectsWrongLengthsSignsAndGarbage)
{
  JointMapping m = JointMapping::racer_default();
  EXPECT_THROW(m.override_with({1, 1}, {}, 4), std::invalid_argument);
  EXPECT_THROW(m.override_with({}, {0.0f}, 4), std::invalid_argument);
  EXPECT_THROW(m.override_with({1, 2, 1, 1}, {}, 4), std::invalid_argument);
  EXPECT_THROW(JointMapping::parse_csv("1,x,3"), std::invalid_argument);
  EXPECT_THROW(JointMapping::parse_csv("1,2abc"), std::invalid_argument);
}

TEST(JointMapping, ParsesTheCsvTheExportWrites)
{
  const auto v = JointMapping::parse_csv("-1,1,-1,-1");
  ASSERT_EQ(v.size(), 4u);
  EXPECT_FLOAT_EQ(v[0], -1.0f);
  const auto w = JointMapping::parse_csv("0.0,0.0,-85.0, 0.5");
  ASSERT_EQ(w.size(), 4u);
  EXPECT_FLOAT_EQ(w[2], -85.0f);
  EXPECT_FLOAT_EQ(w[3], 0.5f);
  EXPECT_TRUE(JointMapping::parse_csv("").empty());
}
