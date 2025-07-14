#include "pinky_hardware_interface/pinky_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pinky_hardware_interface
{
hardware_interface::CallbackReturn PinkySystemHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    RCLCPP_INFO(get_logger(), "Name: %s", info_.name.c_str());
    RCLCPP_INFO(get_logger(), "Number of Joints %zu", info_.joints.size());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as third state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if (info_.gpios.size() != 1)
    {
        RCLCPP_FATAL(
            get_logger(), "PinkySystemHardwareInterface has '%ld' GPIO components, '%d' expected.",
            info_.gpios.size(), 1);
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.gpios[0].command_interfaces.size() != 3)
    {
        RCLCPP_FATAL(
            get_logger(), "GPIO component %s has '%ld' command interfaces, '%d' expected.",
            info_.gpios[0].name.c_str(), info_.gpios[0].command_interfaces.size(), 3);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info_.gpios[0].state_interfaces.size() != 8)
    {
        RCLCPP_FATAL(
            get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
            info_.gpios[0].name.c_str(), info_.gpios[0].state_interfaces.size(), 8);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize Dynamixel SDK
    auto interface_name = info_.hardware_parameters["interface_name"];
    port_ = dynamixel::PortHandler::getPortHandler(interface_name.c_str());
    port_->setBaudRate(1000000);
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    profile_acceleration_ = std::stof(info_.hardware_parameters["profile_acceleration"]);
    profile_velocity_ = std::stof(info_.hardware_parameters["profile_velocity"]);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PinkySystemHardwareInterface::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
    // Initialize the variables

    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        set_state(name, 0.0);
    }
    for (const auto & [name, descr] : joint_command_interfaces_)
    {
        set_command(name, 0.0);
    }
    for (const auto & [name, descr] : gpio_state_interfaces_)
    {
        set_state(name, 0.0);
    }
    for (const auto & [name, descr] : gpio_command_interfaces_)
    {
        set_command(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PinkySystemHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    // Activate the hardware related
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    std::vector<uint8_t> ret_data;
    int ret = packet_->broadcastPing(port_, ret_data);
    if(ret != COMM_SUCCESS)
    {
        RCLCPP_FATAL(get_logger(), "Error broadcadPing@on_activate.");
        RCLCPP_FATAL(get_logger(), "%s", packet_->getTxRxResult(ret));
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Found [%d] dynamixel motors.", (int)ret_data.size());
    if(ret_data.size() != 2)
    {
        RCLCPP_FATAL(get_logger(), "Found [%d] motors, but expected [%d] motors.", (int)ret_data.size(), 2);
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize dynamixel motors

    // Set Velocity Mode
    // Torque Enable
    // LED ON

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PinkySystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    // Deactivate the hardware related
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PinkySystemHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PinkySystemHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pinky_hardware_interface::PinkySystemHardwareInterface, hardware_interface::SystemInterface)