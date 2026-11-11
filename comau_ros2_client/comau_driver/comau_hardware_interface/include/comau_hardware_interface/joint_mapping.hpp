// Copyright 2026 Instinct Robotics
//
// Per-joint relation between the controller's joint values (degrees, as the C5G reports them) and the robot model's
// joints (radians).
//
//   model [deg] = sign * controller [deg] + offset_deg          controller [deg] = sign * (model [deg] - offset_deg)
//
// The built-in table is the one measured on the Racer 5-0.80 URDF (j1 and j4 negated, j3 = -90 - j3, j6 = 34.5 - j6,
// the sign of j6 inverted: with +j6 the TP's R followed j1 - j6 instead of staying constant in a Y translation).
// Other robots can override it with the hardware parameters `joint_signs` and `joint_offsets_deg` (comma separated,
// one value per joint of the arm): the export writes them from the project's robot_driver.json.
// Joints beyond the table (more than 6 axes) are not changed.
//
// Header only and free of ROS dependencies, so it can be unit tested on its own.

#ifndef COMAU_HARDWARE_INTERFACE__JOINT_MAPPING_HPP_
#define COMAU_HARDWARE_INTERFACE__JOINT_MAPPING_HPP_

#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace comau_hardware_interface
{

class JointMapping
{
public:
  /// Mapping built into the driver (Racer 5-0.80).
  static JointMapping racer_default()
  {
    JointMapping m;
    m.sign_ = {-1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f};
    m.offset_deg_ = {0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 34.5f};
    return m;
  }

  /// Replace signs and/or offsets; each list must have `n_joints` values and signs must be +1 or -1.
  /// Empty lists keep what is there. Throws std::invalid_argument with a message for the log.
  void override_with(const std::vector<float> & signs, const std::vector<float> & offsets_deg, size_t n_joints)
  {
    if (!signs.empty() && signs.size() != n_joints) {
      throw std::invalid_argument("joint_signs needs " + std::to_string(n_joints) + " values, got " +
                                  std::to_string(signs.size()));
    }
    if (!offsets_deg.empty() && offsets_deg.size() != n_joints) {
      throw std::invalid_argument("joint_offsets_deg needs " + std::to_string(n_joints) + " values, got " +
                                  std::to_string(offsets_deg.size()));
    }
    for (float s : signs) {
      if (s != 1.0f && s != -1.0f) {
        throw std::invalid_argument("joint_signs values must be 1 or -1");
      }
    }
    if (!signs.empty()) {
      sign_ = signs;
    }
    if (!offsets_deg.empty()) {
      offset_deg_ = offsets_deg;
    }
    sign_.resize(n_joints, 1.0f);
    offset_deg_.resize(n_joints, 0.0f);
  }

  /// "1,-1,1" -> {1,-1,1}. Throws std::invalid_argument on anything that is not a number.
  static std::vector<float> parse_csv(const std::string & text)
  {
    std::vector<float> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
      size_t used = 0;
      float v = 0.0f;
      try {
        v = std::stof(item, &used);
      } catch (const std::exception &) {
        throw std::invalid_argument("'" + item + "' is not a number");
      }
      while (used < item.size() && item[used] == ' ') {
        ++used;
      }
      if (used != item.size()) {
        throw std::invalid_argument("'" + item + "' is not a number");
      }
      values.push_back(v);
    }
    return values;
  }

  /// Controller value in degrees -> model joint in radians.
  double to_model(size_t index, float controller_deg) const
  {
    const float deg = sign(index) * controller_deg + offset(index);
    return static_cast<double>(deg) * M_PI / 180.0;
  }

  /// Model joint in radians -> controller value in degrees.
  float to_controller(size_t index, double model_rad) const
  {
    const float deg = static_cast<float>(model_rad * 180.0 / M_PI);
    return sign(index) * (deg - offset(index));
  }

  float sign(size_t index) const {return index < sign_.size() ? sign_[index] : 1.0f;}
  float offset(size_t index) const {return index < offset_deg_.size() ? offset_deg_[index] : 0.0f;}

private:
  std::vector<float> sign_;
  std::vector<float> offset_deg_;
};

}  // namespace comau_hardware_interface

#endif  // COMAU_HARDWARE_INTERFACE__JOINT_MAPPING_HPP_
