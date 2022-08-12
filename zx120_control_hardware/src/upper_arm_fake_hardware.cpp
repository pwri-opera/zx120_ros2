#include <cstdio>

#include "zx120_control/upper_arm_fake_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// 
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
// 
namespace zx120_control_hardware
{
  hardware_interface::CallbackReturn Zx120UpperArmPositionHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("uac_fake_hw");
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
    node_thread_ = std::thread([this]()
                               { rclcpp::spin(node_); });
    joint_state_msg_.header.frame_id = "joint-state data";
    joint_state_msg_.name.resize(info_.joints.size());
    joint_state_msg_.position.resize(info_.joints.size(), 0);
    joint_state_msg_.velocity.resize(info_.joints.size(), 0);
    joint_state_msg_.effort.resize(info_.joints.size(), 0);
    for (int i = 0; i < (int)info_.joints.size(); i++)
    {
      joint_state_msg_.name[i] = info_.joints[i].name;
    }

    // position_cmds_.resize(info_.joints.size(), 0);

    hw_states_.resize(info_.joints.size(), 0);
    hw_commands_.resize(info_.joints.size(), 0);


    // input initial pose here
    hw_commands_[0] = 0;
    hw_commands_[1] = 0;
    hw_commands_[2] = 1;
    hw_commands_[3] = 0;
    hw_commands_[4] = 0;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // Zx120UpperArmPositionHardware has exactly one state and command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx120UpperArmPositionHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      //command interface must be position
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx120UpperArmPositionHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx120UpperArmPositionHardware"),
            "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx120UpperArmPositionHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

  //
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //     "topic", 10, std::bind(&Zx120UpperArmPositionHardware::topic_callback, this, _1));

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Zx120UpperArmPositionHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // for (uint i = 0; i < hw_states_.size(); i++)
    // {
    //   hw_states_[i] = 0;
    //   hw_commands_[i] = 0;
    // }

    RCLCPP_INFO(rclcpp::get_logger("Zx120UpperArmPositionHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Zx120UpperArmPositionHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
  }
  std::vector<hardware_interface::CommandInterface>
  Zx120UpperArmPositionHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn Zx120UpperArmPositionHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    // for (uint i = 0; i < hw_states_.size(); i++)
    // {
    //   hw_commands_[i] = hw_states_[i];
    // }

    RCLCPP_INFO(rclcpp::get_logger("Zx120UpperArmPositionHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Zx120UpperArmPositionHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::return_type Zx120UpperArmPositionHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // joint_state_msg_.header.stamp = node_->get_clock()->now();
    // joint_state_pub_->publish(joint_state_msg_);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Zx120UpperArmPositionHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    for (int i = 0; i < (int)hw_commands_.size(); i++)
    {
      hw_states_[i] = hw_commands_[i];
      joint_state_msg_.position[i] = hw_commands_[i];
      // RCLCPP_INFO(rclcpp::get_logger("ros_control"), "hw_commands_[%d]:%f", i, hw_commands_[i]);
    }
    // for (int i = 0; i < velocity_cmds_.size(); i++)
    // {
    //   velocity_states_[i] = velocity_cmds_[i];
    //   joint_state_msg_.velocity[i] = velocity_cmds_[i];
    // }

    return hardware_interface::return_type::OK;
  }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    zx120_control_hardware::Zx120UpperArmPositionHardware, hardware_interface::SystemInterface)
