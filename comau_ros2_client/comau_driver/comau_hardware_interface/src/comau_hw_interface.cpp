/**
 * @file comau_hw_interface.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes hardware interface
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include "comau_hardware_interface/comau_hw_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "comau_hardware_interface/joint_mapping.hpp"
#include "comau_tcp_interface/utils/custom_data_type.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace comau_hardware_interface
{

// stsSelector bits published by the controller state server (see state_server.pdl)
static constexpr uint32_t kStsDriveOn = 0x8;
static constexpr uint32_t kStsStart = 0x10;

CallbackReturn ComauHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read hardware parameters
  robot_ip_ = "192.168.2.1";
  if (info_.hardware_parameters.find("robot_ip") != info_.hardware_parameters.end()) {
    robot_ip_ = info_.hardware_parameters.at("robot_ip");
  }
  state_port_ = "1104";
  if (info_.hardware_parameters.find("state_port") != info_.hardware_parameters.end()) {
    state_port_ = info_.hardware_parameters.at("state_port");
  }
  robot_port_ = "1105";
  if (info_.hardware_parameters.find("robot_port") != info_.hardware_parameters.end()) {
    robot_port_ = info_.hardware_parameters.at("robot_port");
  }
  arm1_port_ = "1101";
  if (info_.hardware_parameters.find("arm1_port") != info_.hardware_parameters.end()) {
    arm1_port_ = info_.hardware_parameters.at("arm1_port");
  }
  if (info_.hardware_parameters.find("fly_dist") != info_.hardware_parameters.end()) {
    try {
      fly_dist_ = std::stof(info_.hardware_parameters.at("fly_dist"));
    } catch (...) {}
  }
  if (info_.hardware_parameters.find("fly_lin_velocity") != info_.hardware_parameters.end()) {
    try {
      fly_lin_velocity_ = std::stof(info_.hardware_parameters.at("fly_lin_velocity"));
    } catch (...) {}
  }

  if (info_.hardware_parameters.find("fly_speed_adaptive") != info_.hardware_parameters.end()) {
    const std::string value = info_.hardware_parameters.at("fly_speed_adaptive");
    fly_speed_adaptive_ = !(value == "false" || value == "False" || value == "0");
  }
  if (info_.hardware_parameters.find("fly_override_compensation") != info_.hardware_parameters.end()) {
    const std::string value = info_.hardware_parameters.at("fly_override_compensation");
    fly_override_comp_ = !(value == "false" || value == "False" || value == "0");
  }
  if (info_.hardware_parameters.find("fly_speed_margin") != info_.hardware_parameters.end()) {
    try {
      fly_speed_margin_ = std::clamp(std::stof(info_.hardware_parameters.at("fly_speed_margin")), 1.0f, 5.0f);
    } catch (...) {}
  }
  if (info_.hardware_parameters.find("fly_speed_gain") != info_.hardware_parameters.end()) {
    try {
      fly_speed_gain_ = std::clamp(std::stof(info_.hardware_parameters.at("fly_speed_gain")), 0.0f, 20.0f);
    } catch (...) {}
  }
  if (info_.hardware_parameters.find("fly_speed_min") != info_.hardware_parameters.end()) {
    try {
      fly_speed_min_ = std::clamp(std::stof(info_.hardware_parameters.at("fly_speed_min")), 0.001f, 0.5f);
    } catch (...) {}
  }
  if (info_.hardware_parameters.find("fly_min_step") != info_.hardware_parameters.end()) {
    try {
      fly_min_step_ = std::clamp(std::stof(info_.hardware_parameters.at("fly_min_step")), 0.0f, 0.1f);
    } catch (...) {}
  }
  if (info_.hardware_parameters.find("fly_speed_slew") != info_.hardware_parameters.end()) {
    try {
      fly_speed_slew_ = std::clamp(std::stof(info_.hardware_parameters.at("fly_speed_slew")), 0.0f, 50.0f);
    } catch (...) {}
  }
  if (info_.hardware_parameters.find("fk_base_link") != info_.hardware_parameters.end()) {
    fk_base_link_ = info_.hardware_parameters.at("fk_base_link");
  }
  if (info_.hardware_parameters.find("fk_tip_link") != info_.hardware_parameters.end()) {
    fk_tip_link_ = info_.hardware_parameters.at("fk_tip_link");
  }

  if (info_.hardware_parameters.find("max_resume_deviation") != info_.hardware_parameters.end()) {
    try {
      max_resume_deviation_ = std::stod(info_.hardware_parameters.at("max_resume_deviation"));
    } catch (...) {}
  }

  if (info_.hardware_parameters.find("stop_cancel_controller") != info_.hardware_parameters.end()) {
    const std::string value = info_.hardware_parameters.at("stop_cancel_controller");
    stop_cancel_controller_ = !(value == "false" || value == "False" || value == "0");
  }
  if (info_.hardware_parameters.find("stop_cancel_action") != info_.hardware_parameters.end()) {
    stop_cancel_action_ = info_.hardware_parameters.at("stop_cancel_action");
  }
  if (info_.hardware_parameters.find("stop_resume_deviation") != info_.hardware_parameters.end()) {
    try {
      stop_resume_deviation_ = std::max(0.0, std::stod(info_.hardware_parameters.at("stop_resume_deviation")));
    } catch (...) {}
  }

  if (info_.hardware_parameters.find("speed_override_percent") != info_.hardware_parameters.end()) {
    try {
      requested_override_ = std::clamp(std::stoi(info_.hardware_parameters.at("speed_override_percent")), 1, 100);
    } catch (...) {}
  }
  speed_scaling_ = requested_override_.load() / 100.0;

  size_t n_joints = info_.joints.size();

  // Controller <-> model joint mapping: built-in (Racer 5-0.80) unless the export overrides it for this robot
  joint_mapping_ = JointMapping::racer_default();
  {
    const auto & params = info_.hardware_parameters;
    const bool has_signs = params.find("joint_signs") != params.end();
    const bool has_offsets = params.find("joint_offsets_deg") != params.end();
    if (has_signs || has_offsets) {
      try {
        joint_mapping_.override_with(
          has_signs ? JointMapping::parse_csv(params.at("joint_signs")) : std::vector<float>{},
          has_offsets ? JointMapping::parse_csv(params.at("joint_offsets_deg")) : std::vector<float>{},
          n_joints);
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
                    "Joint mapping overridden by the hardware parameters (joint_signs=%s, joint_offsets_deg=%s)",
                    has_signs ? params.at("joint_signs").c_str() : "built-in",
                    has_offsets ? params.at("joint_offsets_deg").c_str() : "built-in");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("ComauHardwareInterface"), "Invalid joint mapping: %s", e.what());
        return CallbackReturn::ERROR;
      }
    }
  }

  joint_position_.assign(n_joints, 0.0);
  joint_velocities_.assign(n_joints, 0.0);
  joint_position_command_.assign(n_joints, 0.0);
  joint_velocities_command_.assign(n_joints, 0.0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  // Not fatal: without forward kinematics every MOVEFLY step is sent with the fixed fly_lin_velocity
  initForwardKinematics();

  RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
              "ComauHardwareInterface initialized for robot IP: %s (joints: %zu)",
              robot_ip_.c_str(), n_joints);
  return CallbackReturn::SUCCESS;
}

bool ComauHardwareInterface::initForwardKinematics()
{
  fk_ready_ = false;
  KDL::Tree tree;
  if (info_.original_xml.empty() || !kdl_parser::treeFromString(info_.original_xml, tree)) {
    RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                "Adaptive MOVEFLY speed unavailable: cannot build the kinematic tree from the URDF. "
                "Using the fixed fly_lin_velocity.");
    return false;
  }
  const std::string base = fk_base_link_.empty() ? tree.getRootSegment()->first : fk_base_link_;
  if (!tree.getChain(base, fk_tip_link_, fk_chain_)) {
    RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                "Adaptive MOVEFLY speed unavailable: no chain '%s' -> '%s' in the URDF "
                "(set the fk_base_link / fk_tip_link parameters). Using the fixed fly_lin_velocity.",
                base.c_str(), fk_tip_link_.c_str());
    return false;
  }

  fk_joint_index_.clear();
  for (const auto & segment : fk_chain_.segments) {
    const KDL::Joint & joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::Fixed) {
      continue;
    }
    auto it = std::find_if(info_.joints.begin(), info_.joints.end(),
                           [&joint](const hardware_interface::ComponentInfo & j) { return j.name == joint.getName(); });
    if (it == info_.joints.end()) {
      RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                  "Adaptive MOVEFLY speed unavailable: chain joint '%s' is not a joint of this hardware. "
                  "Using the fixed fly_lin_velocity.", joint.getName().c_str());
      return false;
    }
    fk_joint_index_.push_back(static_cast<size_t>(std::distance(info_.joints.begin(), it)));
  }

  fk_q_.resize(fk_chain_.getNrOfJoints());
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(fk_chain_);
  fk_ready_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
              "Adaptive MOVEFLY speed: forward kinematics '%s' -> '%s' (%u joints), margin %.2f, gain %.2f 1/s, "
              "min %.3f m/s, slew %.2f m/s^2, %s", base.c_str(), fk_tip_link_.c_str(), fk_chain_.getNrOfJoints(),
              fly_speed_margin_.load(), fly_speed_gain_.load(), fly_speed_min_.load(), fly_speed_slew_.load(),
              fly_speed_adaptive_.load() ? "enabled" : "disabled");
  return true;
}

bool ComauHardwareInterface::tcpPosition(const std::vector<double> & q, KDL::Vector & p)
{
  for (size_t i = 0; i < fk_joint_index_.size(); ++i) {
    if (fk_joint_index_[i] >= q.size()) {
      return false;
    }
    fk_q_(i) = q[fk_joint_index_[i]];
  }
  if (fk_solver_->JntToCart(fk_q_, fk_frame_) < 0) {
    return false;
  }
  p = fk_frame_.p;
  return true;
}

float ComauHardwareInterface::adaptiveFlySpeed(double dt)
{
  const float cap = fly_lin_velocity_.load();
  if (!fly_speed_adaptive_.load() || !fk_ready_) {
    return cap;
  }

  KDL::Vector tcp_cmd;
  KDL::Vector tcp_meas;
  if (!tcpPosition(joint_position_command_, tcp_cmd) || !tcpPosition(joint_position_, tcp_meas)) {
    have_prev_tcp_ = false;
    return cap;
  }

  // Speed of the commanded trajectory since the previous step, plus a term that recovers the lag
  double v_traj = 0.0;
  if (have_prev_tcp_ && dt > 1e-6) {
    v_traj = (tcp_cmd - prev_tcp_cmd_).Norm() / dt;
  }
  prev_tcp_cmd_ = tcp_cmd;
  have_prev_tcp_ = true;

  const double lag = (tcp_cmd - tcp_meas).Norm();
  // Speed the robot must have (reference speed plus lag recovery) ...
  double v = fly_speed_margin_.load() * v_traj + fly_speed_gain_.load() * lag;
  // ... asked as the speed it has at 100%: the override the controller is applying scales it back down. `sent_override_`
  // is the percentage the controller was told (-1: none sent yet, i.e. 100%).
  if (fly_override_comp_.load() && sent_override_ > 0 && sent_override_ < 100) {
    v /= std::max(0.01, sent_override_ / 100.0);
  }
  const float floor = std::min(fly_speed_min_.load(), cap);
  float speed = std::clamp(static_cast<float>(v), floor, cap);

  // Ramp the speed limit instead of jumping with the lag
  const float slew = fly_speed_slew_.load();
  if (slew > 0.0f && have_prev_fly_speed_ && dt > 1e-6) {
    const float step = slew * static_cast<float>(dt);
    speed = std::clamp(std::clamp(speed, prev_fly_speed_ - step, prev_fly_speed_ + step), floor, cap);
  }
  prev_fly_speed_ = speed;
  have_prev_fly_speed_ = true;
  return speed;
}

bool ComauHardwareInterface::stepFarEnough(bool command_still)
{
  const float min_step = fly_min_step_.load();
  if (min_step <= 0.0f || !have_last_sent_ || last_sent_cmd_.size() != joint_position_command_.size()) {
    return true;
  }
  double max_joint = 0.0;
  for (size_t i = 0; i < joint_position_command_.size(); ++i) {
    max_joint = std::max(max_joint, std::fabs(joint_position_command_[i] - last_sent_cmd_[i]));
  }
  // Holding still (same target as before) or end of a motion: stream as before
  if (max_joint < 1e-6 || command_still) {
    return true;
  }
  if (max_joint >= kMinStepJoint || time_since_send_ >= kMaxStepInterval) {
    return true;
  }
  KDL::Vector tcp;
  if (fk_ready_ && have_last_sent_tcp_ && tcpPosition(joint_position_command_, tcp)) {
    return (tcp - last_sent_tcp_).Norm() >= min_step;
  }
  return false;
}

void ComauHardwareInterface::onRobotStopped()
{
  stop_latched_ = true;
  // The CANCEL itself is sent from write(), retried until the command link takes it
  stop_cancel_pending_ = stop_cancel_controller_;

  if (!jtc_cancel_client_) {
    return;
  }
  if (!jtc_cancel_client_->service_is_ready()) {
    RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                "Cannot cancel the trajectory goal: %s/_action/cancel_goal is not available",
                stop_cancel_action_.c_str());
    return;
  }
  // Zero goal id and zero stamp: cancel every goal of the action server
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  jtc_cancel_client_->async_send_request(
    request, [action = stop_cancel_action_](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
      const auto response = future.get();
      RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                  "Robot stopped: %zu trajectory goal(s) cancelled on %s (return code %d)",
                  response->goals_canceling.size(), action.c_str(), response->return_code);
    });
}

CallbackReturn ComauHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_client_ = std::make_shared<comau_tcp_interface::StateClient>();
  comau_tcp_interface::ComauTcpInterfaceParameters state_params;
  state_params.server_ip_address = robot_ip_;
  state_params.server_port = state_port_;
  state_params.log_tag = "comau_hw_state";
  if (!state_client_->initialize(state_params)) {
    RCLCPP_ERROR(rclcpp::get_logger("ComauHardwareInterface"), "Failed to initialize StateClient");
    return CallbackReturn::ERROR;
  }

  robot_client_ = std::make_shared<comau_tcp_interface::RobotClient>();
  comau_tcp_interface::ComauTcpInterfaceParameters robot_params;
  robot_params.server_ip_address = robot_ip_;
  robot_params.server_port = robot_port_;
  robot_params.log_tag = "comau_hw_robot";
  if (!robot_client_->initialize(robot_params)) {
    RCLCPP_ERROR(rclcpp::get_logger("ComauHardwareInterface"), "Failed to initialize RobotClient");
    return CallbackReturn::ERROR;
  }

  arm1_client_ = std::make_shared<comau_tcp_interface::RobotClient>();
  comau_tcp_interface::ComauTcpInterfaceParameters arm1_params;
  arm1_params.server_ip_address = robot_ip_;
  arm1_params.server_port = arm1_port_;
  arm1_params.log_tag = "comau_hw_arm1";
  if (!arm1_client_->initialize(arm1_params)) {
    RCLCPP_ERROR(rclcpp::get_logger("ComauHardwareInterface"), "Failed to initialize Arm1Client");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
              "Comau TCP clients configured for IP %s (ports: state=%s, robot=%s, arm1=%s)",
              robot_ip_.c_str(), state_port_.c_str(), robot_port_.c_str(), arm1_port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ComauHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
              "Connecting to Comau controller at %s...", robot_ip_.c_str());

  if (!state_client_ || !robot_client_ || !arm1_client_) {
    RCLCPP_ERROR(rclcpp::get_logger("ComauHardwareInterface"), "Clients not configured");
    return CallbackReturn::ERROR;
  }

  bool success = true;
  state_client_->openStateThread(true, success);
  robot_client_->openRobotThread(true, success);
  arm1_client_->openHandlerThread(true, success);

  // Poll for initial state packet (up to 3 seconds)
  auto start_time = std::chrono::steady_clock::now();
  bool got_initial_state = false;
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start_time).count() < 3000)
  {
    if (state_client_->isConnected()) {
      comau_tcp_interface::utils::MessagePackage msg(state_client_->getRecvRecipe());
      if (state_client_->getLastMessage(msg)) {
        comau_tcp_interface::utils::vector10f_t joints_float;
        if (msg.getData("joint_position", joints_float)) {
          for (size_t i = 0; i < joint_position_.size() && i < joints_float.size(); ++i) {
            double rad = joint_mapping_.to_model(i, joints_float[i]);
            joint_position_[i] = rad;
            joint_position_command_[i] = rad;
          }
          got_initial_state = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (got_initial_state && joint_position_.size() >= 6) {
    RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
                "Initial Comau joint positions synchronized: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                joint_position_[0], joint_position_[1], joint_position_[2],
                joint_position_[3], joint_position_[4], joint_position_[5]);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                "Connected to %s, telemetry packet pending...", robot_ip_.c_str());
  }

  robot_ready_ = false;
  streaming_ = false;
  stop_latched_ = false;
  stop_cancel_pending_ = false;
  sent_override_ = -1;
  speed_scaling_ = requested_override_.load() / 100.0;

  // Runtime changes of the override (1..100 %)
  if (auto node = get_node()) {
    override_sub_ = node->create_subscription<std_msgs::msg::Int32>(
      "/comau/speed_override", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        const int percent = std::clamp(static_cast<int>(msg->data), 1, 100);
        if (percent != msg->data) {
          RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                      "Speed override %d out of range, using %d%%", msg->data, percent);
        }
        requested_override_ = percent;
      });

    // Generic (non vendor specific) speed scaling command, 0..1: what the planner publishes.
    // Latched, so a driver started later still gets the current value.
    scaling_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      "/speed_scaling_command", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        const double fraction = std::isfinite(msg->data) ? std::clamp(msg->data, 0.01, 1.0) : 1.0;
        if (fraction != msg->data) {
          RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                      "Speed scaling %.3f out of range, using %.2f", msg->data, fraction);
        }
        requested_override_ = static_cast<int>(std::lround(fraction * 100.0));
      });

    decimation_sub_ = node->create_subscription<std_msgs::msg::Int32>(
      "/comau/stream_decimation", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        const int n = std::clamp(static_cast<int>(msg->data), 1, 20);
        send_every_ = n;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
                    "Sending one MOVEFLY every %d control cycles", n);
      });

    // MOVEFLY tuning at runtime (no restart needed)
    fly_vel_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/comau/fly_lin_velocity", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        const float v = std::clamp(msg->data, 0.005f, 2.0f);
        fly_lin_velocity_ = v;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "MOVEFLY speed cap set to %.3f m/s", v);
      });
    fly_adaptive_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/comau/fly_speed_adaptive", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        fly_speed_adaptive_ = msg->data;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "Adaptive MOVEFLY speed %s",
                    !msg->data ? "off (fixed fly_lin_velocity)" :
                    fk_ready_ ? "on" : "requested but unavailable (no forward kinematics)");
      });
    fly_comp_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/comau/fly_override_compensation", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        fly_override_comp_ = msg->data;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "MOVEFLY speed compensation of the speed override %s",
                    msg->data ? "on" : "off");
      });
    fly_margin_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/comau/fly_speed_margin", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        const float m = std::clamp(msg->data, 1.0f, 5.0f);
        fly_speed_margin_ = m;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "Adaptive MOVEFLY speed margin set to %.2f", m);
      });
    fly_min_step_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/comau/fly_min_step", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        const float d = std::clamp(static_cast<float>(msg->data), 0.0f, 0.1f);
        fly_min_step_ = d;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "MOVEFLY minimum step set to %.1f mm%s", 1000.0f * d,
                    d > 0.0f ? "" : " (off: a step every cycle)");
      });
    fly_slew_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/comau/fly_speed_slew", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        const float r = std::clamp(static_cast<float>(msg->data), 0.0f, 50.0f);
        fly_speed_slew_ = r;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "Adaptive MOVEFLY speed slew set to %.2f m/s^2%s", r,
                    r > 0.0f ? "" : " (off)");
      });
    fly_gain_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/comau/fly_speed_gain", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        const float g = std::clamp(msg->data, 0.0f, 20.0f);
        fly_speed_gain_ = g;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "Adaptive MOVEFLY lag gain set to %.2f 1/s", g);
      });
    fly_speed_pub_ = node->create_publisher<std_msgs::msg::Float32>(
      "/comau/fly_lin_velocity_sent", rclcpp::SystemDefaultsQoS());
    fly_speed_rt_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>>(fly_speed_pub_);
    fly_dist_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "/comau/fly_dist", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        const float d = std::clamp(msg->data, 0.0f, 100.0f);
        fly_dist_ = d;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "MOVEFLY fly distance set to %.3f", d);
      });

    if (!stop_cancel_action_.empty()) {
      jtc_cancel_client_ = node->create_client<action_msgs::srv::CancelGoal>(
        stop_cancel_action_ + "/_action/cancel_goal");
      // Same QoS as the action server status publisher (the last status array is latched)
      jtc_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
        stop_cancel_action_ + "/_action/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
          using action_msgs::msg::GoalStatus;
          for (const auto & status : msg->status_list) {
            const auto & id = status.goal_info.goal_id.uuid;
            if (status.status == GoalStatus::STATUS_ACCEPTED || status.status == GoalStatus::STATUS_EXECUTING) {
              jtc_active_goals_.insert(id);
            } else if (jtc_active_goals_.erase(id) > 0 && status.status == GoalStatus::STATUS_ABORTED) {
              // Only goals seen running: the latched status of old goals must not cancel anything
              jtc_abort_pending_ = true;
            }
          }
        });
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                "No node available: speed override can only be set with the speed_override_percent parameter");
  }
  is_connected_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ComauHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_connected_ = false;
  robot_ready_ = false;
  streaming_ = false;
  override_sub_.reset();
  scaling_sub_.reset();
  decimation_sub_.reset();
  fly_vel_sub_.reset();
  fly_dist_sub_.reset();
  fly_adaptive_sub_.reset();
  fly_comp_sub_.reset();
  fly_margin_sub_.reset();
  fly_gain_sub_.reset();
  fly_slew_sub_.reset();
  fly_min_step_sub_.reset();
  fly_speed_rt_pub_.reset();
  fly_speed_pub_.reset();
  jtc_cancel_client_.reset();
  jtc_status_sub_.reset();
  if (state_client_) {
    state_client_->openStateThread(false, true);
  }
  if (robot_client_) {
    robot_client_->openRobotThread(false, true);
  }
  if (arm1_client_) {
    arm1_client_->openHandlerThread(false, true);
  }
  RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"), "ComauHardwareInterface deactivated.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ComauHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  // Optional gpio <gpio name="speed_scaling"><state_interface name="speed_scaling_factor"/></gpio>
  for (const auto & gpio : info_.gpios)
  {
    for (const auto & state_if : gpio.state_interfaces)
    {
      if (state_if.name == "speed_scaling_factor") {
        state_interfaces.emplace_back(gpio.name, state_if.name, &speed_scaling_);
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ComauHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type ComauHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!is_connected_ || !state_client_) {
    return return_type::OK;
  }

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  // Time scaling follows the speed override requested from the controller
  speed_scaling_ = requested_override_.load() / 100.0;

  bool ready = false;
  if (state_client_->isConnected()) {
    comau_tcp_interface::utils::MessagePackage msg(state_client_->getRecvRecipe());
    if (state_client_->getLastMessage(msg)) {
      comau_tcp_interface::utils::vector10f_t joints_float;
      if (msg.getData("joint_position", joints_float)) {
        for (size_t i = 0; i < joint_position_.size() && i < joints_float.size(); ++i) {
          joint_position_[i] = joint_mapping_.to_model(i, joints_float[i]);
        }
      }

      uint32_t sts_selector = 0;
      if (msg.getData("stsSelector", sts_selector)) {
        ready = (sts_selector & (kStsDriveOn | kStsStart)) == (kStsDriveOn | kStsStart);
      }

      char status = 'R';
      if (msg.getData("robot_status", status) && status == 'E') {
        uint32_t error_val = 0;
        msg.getData("error_value", error_val);
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"),
                              steady_clock, 2000,
                              "Comau robot status 'E' (error/alarm) – code: 0x%05X. "
                              "Clear alarms on controller to resume motion. "
                              "Hardware interface stays active.", error_val);
        // Do NOT return ERROR: deactivating here makes recovery impossible from ROS.
        // Joint state positions are still published from last good packet.
      }
    }
  }

  if (ready != robot_ready_) {
    if (ready) {
      RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
                  "Robot is in Drive ON and START: motion commands enabled.");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                  "Robot is not in Drive ON and START (or state link is down): "
                  "motion commands are paused until it is ready again.");
      onRobotStopped();
    }
    robot_ready_ = ready;
    streaming_ = false;
  }

  if (!robot_ready_) {
    // Keep the command aligned with the real robot so nothing jumps when it becomes ready again
    for (size_t i = 0; i < joint_position_command_.size() && i < joint_position_.size(); ++i) {
      joint_position_command_[i] = joint_position_[i];
    }
  }

  return return_type::OK;
}

return_type ComauHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!is_connected_ || !arm1_client_) {
    return return_type::OK;
  }

  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  // Speed override: sent on the command channel (port 1105) on every (re)connection and on every change
  if (robot_client_) {
    if (!robot_client_->isConnected()) {
      sent_override_ = -1;
    } else if (requested_override_.load() != sent_override_) {
      const int requested = requested_override_.load();
      bool sent = false;
      try {
        sent = robot_client_->sendSpeedOverrideMessage(static_cast<uint32_t>(requested));
      } catch (...) {}
      if (sent) {
        sent_override_ = requested;
        RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
                    "Arm speed override sent to the controller: %d%%", requested);
      } else {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 2000,
                             "Failed to send the arm speed override to the controller");
      }
    }
  }

  // Trajectory goal aborted: stop where the robot is instead of finishing the queued steps and coming back
  if (jtc_abort_pending_.exchange(false) && stop_cancel_controller_) {
    stop_latched_ = true;
    stop_cancel_pending_ = true;
    streaming_ = false;
    RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                "Trajectory goal aborted: cancelling the motions queued on the controller");
  }

  // After a stop, drop the MOVEFLY steps still queued on the controller (robot_server is NOHOLD, so it
  // takes the command in DRIVE OFF too): otherwise they restart by themselves at the next START
  if (stop_cancel_pending_ && robot_client_ && robot_client_->isConnected()) {
    bool sent = false;
    try {
      sent = robot_client_->sendCancelMessage();
    } catch (...) {}
    if (sent) {
      stop_cancel_pending_ = false;
      RCLCPP_WARN(rclcpp::get_logger("ComauHardwareInterface"),
                  "Robot stopped: motions queued on the controller cancelled");
    } else {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 2000,
                           "Robot stopped: failed to cancel the motions queued on the controller, retrying");
    }
  }

  if (!arm1_client_->isConnected()) {
    streaming_ = false;
    return return_type::OK;
  }

  // The controller rejects motion unless Drive ON and START are set (error 43008, latched until reset)
  if (!robot_ready_) {
    return return_type::OK;
  }

  // Never stream new steps on top of a controller queue that still has to be cancelled
  if (stop_cancel_pending_) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 2000,
                         "Not resuming motion: the controller queue has not been cancelled yet");
    return return_type::OK;
  }

  // Never stream motion before the controller has the requested override (it would run at the old one)
  if (sent_override_ != requested_override_.load()) {
    streaming_ = false;
    return return_type::OK;
  }

  // When streaming (re)starts, make sure the controller command is close to the real robot
  if (!streaming_) {
    double max_dev = 0.0;
    for (size_t i = 0; i < joint_position_command_.size() && i < joint_position_.size(); ++i) {
      max_dev = std::max(max_dev, std::fabs(joint_position_command_[i] - joint_position_[i]));
    }
    // After a stop the residual trajectory is dropped: the robot may only resume where it stands
    const double limit = stop_latched_ ? stop_resume_deviation_ : max_resume_deviation_;
    if (max_dev > limit) {
      if (stop_latched_) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 2000,
                             "Not resuming motion after the stop: command differs from the robot by %.3f rad "
                             "(limit %.3f). The residual trajectory is dropped: send a new goal.",
                             max_dev, limit);
      } else {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 2000,
                             "Not resuming motion: command differs from the robot by %.3f rad (limit %.3f). "
                             "Cancel the active trajectory or restart the joint trajectory controller.",
                             max_dev, limit);
      }
      return return_type::OK;
    }
    if (stop_latched_) {
      stop_latched_ = false;
      RCLCPP_INFO(rclcpp::get_logger("ComauHardwareInterface"),
                  "Resuming motion after the stop (command %.4f rad from the robot)", max_dev);
    }
    streaming_ = true;
    have_prev_tcp_ = false;
    have_prev_fly_speed_ = false;
    have_last_sent_ = false;
    time_since_send_ = 0.0;
  }

  // Time between two MOVEFLY steps, for the speed of the commanded trajectory (adaptive speed)
  time_since_send_ += period.seconds();

  // The command stopped changing since the previous cycle (end of a motion, or holding)
  bool command_still = prev_cycle_cmd_.size() == joint_position_command_.size();
  for (size_t i = 0; command_still && i < joint_position_command_.size(); ++i) {
    command_still = std::fabs(joint_position_command_[i] - prev_cycle_cmd_[i]) < 1e-7;
  }
  prev_cycle_cmd_ = joint_position_command_;

  // Optional decimation of the stream: regular, sparser targets instead of every 10 ms cycle
  const int every = send_every_.load();
  if (every > 1 && (send_cycle_++ % static_cast<uint32_t>(every)) != 0) {
    return return_type::OK;
  }

  // Targets too close to the previous one are skipped (see fly_min_step)
  if (!stepFarEnough(command_still)) {
    return return_type::OK;
  }

  const float lin_velocity = adaptiveFlySpeed(time_since_send_);
  time_since_send_ = 0.0;

  comau_tcp_interface::utils::vector6f_t cmd;
  for (size_t i = 0; i < 6; ++i) {
    if (i < joint_position_command_.size()) {
      cmd[i] = joint_mapping_.to_controller(i, joint_position_command_[i]);
    } else {
      cmd[i] = 0.0f;
    }
  }

  bool sent = false;
  try {
    sent = arm1_client_->sendJointMoveFlyMessage(cmd, lin_velocity, fly_dist_.load());
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 1000,
                         "Exception while sending to the Comau controller: %s", e.what());
  } catch (...) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 1000,
                         "Unknown exception while sending to the Comau controller");
  }

  if (sent) {
    last_sent_cmd_ = joint_position_command_;
    have_last_sent_ = true;
    have_last_sent_tcp_ = fk_ready_ && tcpPosition(joint_position_command_, last_sent_tcp_);
  }

  if (sent && fly_speed_rt_pub_) {
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = lin_velocity;
    fly_speed_rt_pub_->try_publish(speed_msg);
  }

  if (!sent) {
    // Returning ERROR would deactivate the whole hardware component and require a ROS restart.
    // The TCP client reconnects by itself, so just pause streaming until the link is back.
    streaming_ = false;
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ComauHardwareInterface"), steady_clock, 1000,
                         "Failed to send joint movefly message to Comau controller at %s "
                         "(motion paused, waiting for reconnection)",
                         robot_ip_.c_str());
  }

  return return_type::OK;
}

}  // namespace comau_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(comau_hardware_interface::ComauHardwareInterface, hardware_interface::SystemInterface)