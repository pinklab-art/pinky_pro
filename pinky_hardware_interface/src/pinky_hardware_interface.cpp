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

    if (info_.gpios[0].command_interfaces.size() != 2)
    {
        RCLCPP_FATAL(
            get_logger(), "GPIO component %s has '%ld' command interfaces, '%d' expected.",
            info_.gpios[0].name.c_str(), info_.gpios[0].command_interfaces.size(), 2);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info_.gpios[0].state_interfaces.size() != 10)
    {
        RCLCPP_FATAL(
            get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
            info_.gpios[0].name.c_str(), info_.gpios[0].state_interfaces.size(), 10);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize Dynamixel SDK
    auto interface_name = info_.hardware_parameters["interface_name"];
    port_ = dynamixel::PortHandler::getPortHandler(interface_name.c_str());
    port_->setBaudRate(1000000);
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    profile_acceleration_ = std::stof(info_.hardware_parameters["profile_acceleration"]);
    profile_velocity_ = std::stof(info_.hardware_parameters["profile_velocity"]);

    last_positions_.resize(2, 0.0);

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

    if(!port_->openPort())
    {
        RCLCPP_FATAL(get_logger(), "Error openPort@on_activate.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    int ret = 0;
    uint8_t dxl_error = 0;

    std::vector<uint8_t> ret_data;
    ret = packet_->broadcastPing(port_, ret_data);
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
    for(size_t id = 1; id <= ret_data.size(); id++)
    {
        ret = packet_->reboot(port_, (uint8_t)id, &dxl_error);
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] reboot@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        ret = packet_->write1ByteTxRx(port_, (uint8_t)id, 11, 1, &dxl_error);  // Operation Mode : Velocity
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] OperationMode@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        ret = packet_->write1ByteTxRx(port_, (uint8_t)id, 64, 1, &dxl_error);  // Torque Enable
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] TorqueEnable@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        ret = packet_->write4ByteTxRx(port_, (uint8_t)id, 108, 0, &dxl_error);  // Profile Acceleration
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] ProfileAcceleration@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        ret = packet_->write1ByteTxRx(port_, (uint8_t)id, 65, 1, &dxl_error);  // LED ON
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] LedOn@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        ret = packet_->write2ByteTxRx(port_, (uint8_t)id, 78, 120, &dxl_error);  // Velocity P Gain
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] LedOn@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        ret = packet_->write2ByteTxRx(port_, (uint8_t)id, 76, 400, &dxl_error);  // Velocity I Gain
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] LedOn@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        //??
        // ret = packet_->write1ByteTxRx(port_, (uint8_t)id, 98, 50, &dxl_error);  // Bus Watchdog: 50 = 1000ms
        // if(ret != COMM_SUCCESS)
        // {
        //     RCLCPP_FATAL(get_logger(), "Error [%d] BusWatchdog@on_activate.", (int)id);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }
    }
    // WatchDog

    bulk_read_ = std::make_shared<dynamixel::GroupBulkRead>(port_, packet_);
    bulk_read_->addParam(1, 64, 83);
    bulk_read_->addParam(2, 64, 83);

    bulk_write_ = std::make_shared<dynamixel::GroupBulkWrite>(port_, packet_);

    // port_->setPacketTimeout(20);
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PinkySystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    // Deactivate the hardware related
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    int ret = 0;
    uint8_t dxl_error = 0;

    for(int id = 1; id < 3; id++)
    {
        ret = packet_->write1ByteTxRx(port_, (uint8_t)id, 64, 0, &dxl_error);  // Torque Disable
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] TorqueDisable@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        ret = packet_->write1ByteTxRx(port_, (uint8_t)id, 65, 0, &dxl_error);  // LED OFF
        if(ret != COMM_SUCCESS)
        {
            RCLCPP_FATAL(get_logger(), "Error [%d] LedOff@on_activate.", (int)id);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PinkySystemHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    int ret = 0;
    ret = bulk_read_->txRxPacket();
    if(ret != COMM_SUCCESS)
    {
        RCLCPP_FATAL(get_logger(), "Error txRxPacket@read.");
        return hardware_interface::return_type::ERROR;
    }

    auto l_cur_vel = (int32_t)bulk_read_->getData(1, 128, 4) * (0.229 * (2.0 * M_PI) / 60.0);
    auto r_cur_vel = (int32_t)bulk_read_->getData(2, 128, 4) * (0.229 * (2.0 * M_PI) / 60.0 * -1);

    set_state("l_wheel_joint/velocity", l_cur_vel);
    set_state("r_wheel_joint/velocity", r_cur_vel);

    last_positions_[0] += l_cur_vel * (static_cast<double>(period.nanoseconds()) / 1e9);
    last_positions_[1] += r_cur_vel * (static_cast<double>(period.nanoseconds()) / 1e9);

    set_state("l_wheel_joint/position", last_positions_[0]);
    set_state("r_wheel_joint/position", last_positions_[1]);

    set_state("l_wheel_joint/effort", bulk_read_->getData(1, 126, 2) / 1000.0);
    set_state("r_wheel_joint/effort", bulk_read_->getData(2, 126, 2) / 1000.0);

    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10, "Position: %4.2f %4.2f %d", last_positions_[0], last_positions_[1], (int32_t)bulk_read_->getData(2, 128, 4));
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PinkySystemHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    uint8_t param_goal_value[4] = {0, 0, 0, 0};

    int32_t l_goal_velocity = (int32_t)(get_command("l_wheel_joint/velocity") / (2.0 * M_PI) * 60.0 / 0.229);
    int32_t r_goal_velocity = (int32_t)(get_command("r_wheel_joint/velocity") / (2.0 * M_PI) * 60.0 / 0.229 * -1);

    param_goal_value[0] = DXL_LOBYTE(DXL_LOWORD(l_goal_velocity));
    param_goal_value[1] = DXL_HIBYTE(DXL_LOWORD(l_goal_velocity));
    param_goal_value[2] = DXL_LOBYTE(DXL_HIWORD(l_goal_velocity));
    param_goal_value[3] = DXL_HIBYTE(DXL_HIWORD(l_goal_velocity));

    bulk_write_->addParam(1, 104, 4, param_goal_value);

    param_goal_value[0] = DXL_LOBYTE(DXL_LOWORD(r_goal_velocity));
    param_goal_value[1] = DXL_HIBYTE(DXL_LOWORD(r_goal_velocity));
    param_goal_value[2] = DXL_LOBYTE(DXL_HIWORD(r_goal_velocity));
    param_goal_value[3] = DXL_HIBYTE(DXL_HIWORD(r_goal_velocity));

    bulk_write_->addParam(2, 104, 4, param_goal_value);

    bulk_write_->txPacket();
    bulk_write_->clearParam();

    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pinky_hardware_interface::PinkySystemHardwareInterface, hardware_interface::SystemInterface)