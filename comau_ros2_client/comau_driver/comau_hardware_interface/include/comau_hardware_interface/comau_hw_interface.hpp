/**
 * @file comau_hw_interface.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes hardware interface
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#ifndef ROS2_COMAU_HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_
#define ROS2_COMAU_HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include "comau_hardware_interface/joint_mapping.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <array>
#include <atomic>
#include <mutex>
#include <set>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

#include "comau_tcp_interface/comau_state_client.hpp"
#include "comau_tcp_interface/comau_robot_client.hpp"
#include "comau_tcp_interface/comau_tcp_interface.hpp"

using hardware_interface::return_type;

#ifndef HARDWARE_INTERFACE_PUBLIC
#define HARDWARE_INTERFACE_PUBLIC
#endif

namespace comau_hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct ComauHardwareInterfaceTestPeer;  // unit tests only (test/test_cancel_on_deactivate.cpp)

class HARDWARE_INTERFACE_PUBLIC ComauHardwareInterface : public hardware_interface::SystemInterface
{
public:
  friend struct ComauHardwareInterfaceTestPeer;
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  // Shutdown / error from the ACTIVE state do not necessarily go through on_deactivate: the queued motion
  // must be cancelled there too (see cancelQueuedMotion)
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  ~ComauHardwareInterface() override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::string robot_ip_{"192.168.2.1"};
  std::string state_port_{"1104"};
  std::string robot_port_{"1105"};
  std::string arm1_port_{"1101"};
  // MOVEFLY fly distance, tunable at runtime on /comau/fly_dist.
  // 5 capped the robot at ~150 mm/s (short steps barely blended); 50 followed 120-720 mm/s (tests 2026-10-13);
  // 100 gave slightly smoother starts at low speed (tests 2026-10-14)
  std::atomic<float> fly_dist_{100.0f};
  // Cap on the linear speed of each MOVEFLY step (m/s). With the adaptive speed off it is the speed sent with
  // every step: it must then stay above the fastest planned TCP speed (Pilz max_trans_vel), otherwise the
  // controller clips the stream and the robot lags behind ROS.
  // Tunable at runtime on /comau/fly_lin_velocity (m/s)
  std::atomic<float> fly_lin_velocity_{1.25f};

  // Adaptive MOVEFLY speed: each step is sent with the TCP speed of the commanded trajectory instead of a
  // fixed cap. A cap far above the trajectory speed makes the controller reach every step early and wait
  // for the next one (stop-and-go tremor); a cap equal to it leaves no room to recover the lag (abort).
  //   lin_spd = clamp(margin * v_traj + gain * |tcp_cmd - tcp_meas|, fly_speed_min, fly_lin_velocity)
  // Runtime topics: /comau/fly_speed_adaptive (Bool), /comau/fly_speed_margin, /comau/fly_speed_gain (Float32).
  // The speed actually sent is published on /comau/fly_lin_velocity_sent.
  std::atomic<bool> fly_speed_adaptive_{true};
  // The controller applies the speed override ($ARM_OVR) on top of the MOVEFLY speed, while the trajectory controller
  // already slows the reference with the speed scaling: the speed asked from the controller is divided by the override
  // it is applying, so the robot ends up at the speed of the (already slowed) reference and neither falls behind nor jumps
  // when the override goes back to 100%. Runtime topic /comau/fly_override_compensation (Bool); hardware parameter
  // fly_override_compensation.
  std::atomic<bool> fly_override_comp_{true};
  std::atomic<float> fly_speed_margin_{1.3f};   // multiplier on the trajectory TCP speed
  std::atomic<float> fly_speed_gain_{4.0f};     // 1/s, lag recovery: extra speed per metre of TCP lag (1 left a ~0.8 s tail)
  std::atomic<float> fly_speed_min_{0.01f};     // m/s, floor (rotation-only steps have ~0 linear speed)
  // Max rate of change of the adaptive speed between two steps (m/s^2, 0 = off): the lag term made the speed
  // limit jump at every start and stop, and the controller followed with abrupt accelerations. Above the
  // trajectory acceleration (Pilz 2.25 m/s^2) so the limit never falls behind the trajectory speed.
  // Runtime topic /comau/fly_speed_slew (Float32); hardware parameter fly_speed_slew.
  std::atomic<float> fly_speed_slew_{3.0f};
  float prev_fly_speed_{0.0f};
  bool have_prev_fly_speed_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fly_slew_sub_;
  // Minimum TCP distance (m) between two MOVEFLY targets (0 = a step every control cycle). At the start of a
  // motion and at low speed the targets were only 1-5 mm ahead of the robot: the controller executed each one as
  // a short move of its own (too short to be blended) and the robot started in stop-and-go. A step is also sent
  // when a joint moved more than kMinStepJoint (rotations), when the command stops (final target) and at least
  // every kMaxStepInterval. Runtime topic /comau/fly_min_step (Float32, m); hardware parameter fly_min_step.
  // Off by default: 10 mm made the start slower (~0.25 s at half the speed) and rougher (test 2026-10-14).
  std::atomic<float> fly_min_step_{0.0f};
  static constexpr double kMinStepJoint = 0.02;       // rad
  static constexpr double kMaxStepInterval = 0.5;     // s
  bool stepFarEnough(bool command_still);
  std::vector<double> last_sent_cmd_;
  std::vector<double> prev_cycle_cmd_;
  KDL::Vector last_sent_tcp_;
  bool have_last_sent_{false};
  bool have_last_sent_tcp_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fly_min_step_sub_;
  std::string fk_base_link_;                    // empty = root of the URDF tree
  std::string fk_tip_link_{"flange"};           // the controller limits the flange speed ($TOOL is zero)

  /// Forward kinematics of the commanded and measured joints (built from the URDF in on_init)
  bool initForwardKinematics();
  bool tcpPosition(const std::vector<double> & q, KDL::Vector & p);
  float adaptiveFlySpeed(double dt);

  bool fk_ready_{false};
  KDL::Chain fk_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::vector<size_t> fk_joint_index_;          // KDL chain joint -> index in info_.joints
  KDL::JntArray fk_q_;
  KDL::Frame fk_frame_;
  bool have_prev_tcp_{false};
  KDL::Vector prev_tcp_cmd_;
  double time_since_send_{0.0};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fly_adaptive_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fly_comp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fly_margin_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fly_gain_sub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> fly_speed_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>> fly_speed_rt_pub_;

  std::shared_ptr<comau_tcp_interface::StateClient> state_client_;
  std::shared_ptr<comau_tcp_interface::RobotClient> robot_client_;
  std::shared_ptr<comau_tcp_interface::RobotClient> arm1_client_;

  bool is_connected_{false};

  /// Drive ON and START both set on the controller (stsSelector bits 0x8 and 0x10).
  /// Motion commands are rejected by the controller otherwise (error 43008) and the error stays latched.
  bool robot_ready_{false};
  /// True while commands are being streamed; reset whenever the robot stops being ready or a link drops.
  bool streaming_{false};
  /// Max joint distance (rad) between the controller command and the robot when streaming resumes.
  double max_resume_deviation_{0.5};

  /// Stop handling (E-stop, safety gate, DRIVE OFF, HOLD: the robot stops being ready while connected).
  /// The residual motion is dropped on both sides, so nothing restarts by itself after reset + START:
  ///  - the MOVEFLY steps still queued on the controller are cancelled (robot command 'X', CANCEL ALL);
  ///  - the active goal of the joint trajectory controller is cancelled (it then holds the measured position);
  ///  - streaming resumes only when the command is within stop_resume_deviation of the robot,
  ///    i.e. once the trajectory controller holds where the robot is or a new goal starts from there.
  /// Hardware parameters: stop_cancel_controller (bool), stop_cancel_action (action name, "" = off),
  /// stop_resume_deviation (rad).
  void onRobotStopped();

  /// Disconnect safety: the MOVEFLY steps already queued on the controller keep running when the ROS side goes
  /// away (deactivation, shutdown, error). Sends the CANCEL (robot command 'X', CANCEL ALL on the arm) on the
  /// command link BEFORE the sockets are closed, if something may be queued (streaming active, a step sent since
  /// the last resume, or a stop cancel still pending). Idempotent: once the cancel went through, further calls do
  /// nothing until new steps are streamed. A failure is logged as an error and never blocks the caller (the
  /// controller-side PDL cancels the queue by itself when the motion socket closes). Returns true if nothing had to
  /// be sent or the cancel was sent. Takes write_mutex_.
  bool cancelQueuedMotion(const char * reason);
  /// Serialises write() with cancelQueuedMotion(): the cancel is never interleaved with a MOVEFLY being sent, and
  /// no new MOVEFLY is sent once the cancel has been taken (write() uses try_lock: it never blocks the control loop)
  std::mutex write_mutex_;
  bool stop_cancel_controller_{true};
  std::string stop_cancel_action_{"/joint_trajectory_controller/follow_joint_trajectory"};
  double stop_resume_deviation_{0.01};
  /// Set by a stop, cleared when streaming resumes: the strict stop_resume_deviation applies meanwhile
  bool stop_latched_{false};
  /// The controller queue still has to be cancelled: no motion is streamed until it is
  bool stop_cancel_pending_{false};
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr jtc_cancel_client_;
  /// Abort of a trajectory goal (path tolerance, goal time): the trajectory controller then holds where the robot
  /// was, but the MOVEFLY steps already queued on the controller carry the robot on by a few cm and then back.
  /// An abort seen on <stop_cancel_action>/_action/status cancels the controller queue as a stop does, so the robot
  /// stops where it is and waits for a new goal (stop_resume_deviation applies).
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jtc_status_sub_;
  std::set<std::array<uint8_t, 16>> jtc_active_goals_;   // goals seen accepted/executing (status callback only)
  std::atomic<bool> jtc_abort_pending_{false};

  /// Arm speed override (percent, 1..100) requested by the hardware parameter `speed_override_percent`
  /// or at runtime on the topic /comau/speed_override. It is sent to the controller, which applies it to $ARM_OVR.
  std::atomic<int> requested_override_{100};
  /// Last override value sent to the controller (-1 = nothing sent on the current connection)
  int sent_override_{-1};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr override_sub_;
  /// Vendor-neutral input: speed scaling fraction (0..1) on /speed_scaling_command, same effect as override_sub_
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scaling_sub_;
  // Send one MOVEFLY every N control cycles (1 = every cycle, 100 Hz); tunable on /comau/stream_decimation
  std::atomic<int> send_every_{1};
  uint32_t send_cycle_{0};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr decimation_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fly_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fly_dist_sub_;

  /// Requested override as a 0..1 factor, exported as state interface speed_scaling/speed_scaling_factor
  /// so that the joint trajectory controller stretches the trajectory time and the robot never lags behind.
  double speed_scaling_{1.0};

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  /// Controller <-> model joint mapping (built-in, or the `joint_signs` / `joint_offsets_deg` parameters)
  JointMapping joint_mapping_{JointMapping::racer_default()};
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  // namespace comau_hardware_interface

#endif
