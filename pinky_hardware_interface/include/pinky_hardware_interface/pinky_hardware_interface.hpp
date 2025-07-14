#ifndef PINKY_HARDWARE_INTERFACE__PINKY_HARDWARE_INTERFACE_HPP_
#define PINKY_HARDWARE_INTERFACE__PINKY_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace pinky_hardware_interface
{
    class PinkySystemHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(PinkySystemHardwareInterface)

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            dynamixel::PortHandler* port_;
            dynamixel::PacketHandler* packet_;

            std::shared_ptr<dynamixel::GroupBulkRead> bulk_read_;
            std::shared_ptr<dynamixel::GroupBulkWrite> bulk_write_;

            double profile_acceleration_;
            double profile_velocity_;
            std::vector<double> last_positions_;
    };
}


#endif //PINKY_HARDWARE_INTERFACE__PINKY_HARDWARE_INTERFACE_HPP_