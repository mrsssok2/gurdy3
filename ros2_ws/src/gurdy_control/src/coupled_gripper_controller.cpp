// CoupledGripperController: the two prismatic fingers of the arm's gripper move as ONE rack (the real gripper is one SG90
// servo driving a rack, so both fingers always move symmetrically). Gazebo Harmonic's DART plugin and gz_ros2_control
// 1.2.x here have no mimic-joint support, so the coupling is a stiff spring-damper on the finger DIFFERENCE:
//
//   u      = mean of the two commanded efforts (std_msgs/Float64MultiArray on ~/commands, + = open: same API as the
//            effort_controllers/JointGroupEffortController it replaces)
//   c      = kc * (q_left - q_right) + kd * (qd_left - qd_right)
//   tau_l  = clamp(u - c),   tau_r = clamp(u + c)
//
// The squeeze (sum of the two efforts = 2u) is unchanged; only a sideways slide of the finger pair is resisted
// (overnight 2026-11-27: fingers closed centred at the object width, then slid to 40 / 10 mm during the lift and the
// object stayed on the floor). Gains come from arm.yaml `gripper_coupling` (stability: gurdy_core/tests/test_gripper_coupling.py).
#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace gurdy_control
{
using CallbackReturn = controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

class CoupledGripperController : public controller_interface::ControllerInterface
{
public:
  CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("coupling_kp", 0.0);
    auto_declare<double>("coupling_kd", 0.0);
    auto_declare<double>("effort_limit", 0.0);
    return CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration command_interface_configuration() const override
  {
    InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
    for (const auto & j : joints_) cfg.names.push_back(j + "/effort");
    return cfg;
  }

  InterfaceConfiguration state_interface_configuration() const override
  {
    InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
    for (const auto & j : joints_) {
      cfg.names.push_back(j + "/position");
      cfg.names.push_back(j + "/velocity");
    }
    return cfg;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    kc_ = node->get_parameter("coupling_kp").as_double();
    kd_ = node->get_parameter("coupling_kd").as_double();
    effort_limit_ = node->get_parameter("effort_limit").as_double();
    if (joints_.size() != 2 || kc_ < 0.0 || kd_ < 0.0 || effort_limit_ <= 0.0) {
      RCLCPP_ERROR(node->get_logger(), "need exactly 2 joints, coupling_kp >= 0, coupling_kd >= 0, effort_limit > 0");
      return CallbackReturn::ERROR;
    }
    u_ = pending_ = 0.0;
    has_pending_ = false;
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.empty() || msg->data.size() > 2) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                               "command has %zu values, expected 1 or 2 - ignored", msg->data.size());
          return;
        }
        double u = msg->data[0];
        if (msg->data.size() == 2) u = 0.5 * (msg->data[0] + msg->data[1]);   // one servo: one effort for the rack
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = u;
        has_pending_ = true;
      });
    RCLCPP_INFO(node->get_logger(), "coupled gripper %s + %s, coupling kp=%.1f N/m kd=%.3f N s/m, effort_limit=%.1f N",
                joints_[0].c_str(), joints_[1].c_str(), kc_, kd_, effort_limit_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (size_t i = 0; i < 2; ++i) {
      pos_idx_[i] = vel_idx_[i] = cmd_idx_[i] = SIZE_MAX;
      for (size_t k = 0; k < state_interfaces_.size(); ++k) {
        const auto & si = state_interfaces_[k];
        if (si.get_prefix_name() != joints_[i]) continue;
        if (si.get_interface_name() == "position") pos_idx_[i] = k;
        if (si.get_interface_name() == "velocity") vel_idx_[i] = k;
      }
      for (size_t k = 0; k < command_interfaces_.size(); ++k) {
        if (command_interfaces_[k].get_prefix_name() == joints_[i]) cmd_idx_[i] = k;
      }
      if (pos_idx_[i] == SIZE_MAX || vel_idx_[i] == SIZE_MAX || cmd_idx_[i] == SIZE_MAX) {
        RCLCPP_ERROR(get_node()->get_logger(), "interfaces missing for joint %s", joints_[i].c_str());
        return CallbackReturn::ERROR;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    u_ = 0.0;
    has_pending_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    for (auto & c : command_interfaces_) (void)c.set_value(0.0);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock() && has_pending_) {
        u_ = pending_;
        has_pending_ = false;
      }
    }
    const auto ql = state_interfaces_[pos_idx_[0]].get_optional();
    const auto qr = state_interfaces_[pos_idx_[1]].get_optional();
    const auto vl = state_interfaces_[vel_idx_[0]].get_optional();
    const auto vr = state_interfaces_[vel_idx_[1]].get_optional();
    if (!ql || !qr || !vl || !vr) return controller_interface::return_type::OK;   // keep previous command
    const double c = kc_ * (*ql - *qr) + kd_ * (*vl - *vr);
    (void)command_interfaces_[cmd_idx_[0]].set_value(std::clamp(u_ - c, -effort_limit_, effort_limit_));
    (void)command_interfaces_[cmd_idx_[1]].set_value(std::clamp(u_ + c, -effort_limit_, effort_limit_));
    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joints_;
  double kc_{0.0}, kd_{0.0}, effort_limit_{0.0};
  double u_{0.0}, pending_{0.0};
  bool has_pending_{false};
  std::mutex mutex_;
  size_t pos_idx_[2], vel_idx_[2], cmd_idx_[2];
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

}  // namespace gurdy_control

PLUGINLIB_EXPORT_CLASS(gurdy_control::CoupledGripperController, controller_interface::ControllerInterface)
