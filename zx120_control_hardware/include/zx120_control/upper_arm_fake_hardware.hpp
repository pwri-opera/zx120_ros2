#ifndef UPPER_ARM_CONTROLLER_HPP_
#define UPPER_ARM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace zx120_control_hardware
{
  class Zx120UpperArmPositionHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Zx120UpperArmPositionHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
  private:
    //   void topic_callback(const std_msgs::msg::String &msg);
    //   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
      std::vector<double> hw_commands_;
      std::vector<double> hw_states_;
    //   std::vector<double> position_cmds_;
    //   std::vector<double> velocity_cmds_;
      std::vector<double> position_states_;
      std::vector<double> velocity_states_;

      std::shared_ptr<rclcpp::Node> node_;
      std::thread node_thread_;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
      sensor_msgs::msg::JointState joint_state_msg_;
  };
}
#endif //UPPER_ARM_CONTROLLER_HPP_