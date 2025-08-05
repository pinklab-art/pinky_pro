#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "wiringPiI2C.h"

using realtime_tools::RealtimePublisher;

class PinkySensorADC : public rclcpp::Node
{
    public:
        PinkySensorADC() : Node("pinky_sensor_adc")
        {
            this->declare_parameter<std::string>("interface", "/dev/i2c-1");
            this->declare_parameter<double>("rate", 20.0);

            auto interface = this->get_parameter("interface").get_parameter_value().get<std::string>();

            fd_ = wiringPiI2CSetupInterface(interface.c_str(), 0x08);
            if (fd_ == -1) {
                RCLCPP_FATAL(this->get_logger(), "Failed to init I2C communication.");
                assert(false);
            }

            pub_us_sensor_ = this->create_publisher<sensor_msgs::msg::Range>("us_sensor/range", 10);
            pub_ir_sensor_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("ir_sensor/range", 10);
            pub_batt_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>("batt_state", 10);

            // Timer Loop (100Hz)
            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&PinkySensorADC::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "%s initialized...", this->get_name());
        }
        ~PinkySensorADC() {}

    private:
        void timer_callback()
        {
            uint8_t registers[5] = {0x88, 0xC8, 0x98, 0xD8, 0xF8};
            uint16_t adc_result[5] = {0, 0, 0, 0, 0};

            for(int i = 0; i < 5; i++)
            {
                uint8_t data[2] = {0, };

                wiringPiI2CRawWrite(fd_, &registers[i], 1);
                rclcpp::sleep_for(std::chrono::milliseconds(6));

                wiringPiI2CRawRead(fd_, data, 2);
                adc_result[i] = uint16_t((data[0] << 4)) + uint16_t(data[1] >> 4);
            }

            auto us_result = sensor_msgs::msg::Range();
            us_result.header.stamp = this->now();
            us_result.header.frame_id = "ultrasonic_link";
            us_result.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            us_result.field_of_view = 0.26;
            us_result.min_range = 0.02;
            us_result.max_range = 3.0;
            us_result.range = 1.0 * (adc_result[3] / 4096.0) - 0.03;
            us_result.variance = 0;

            pub_us_sensor_->publish(us_result);


            auto ir_result = std_msgs::msg::UInt16MultiArray();
            ir_result.data.push_back(adc_result[2]);
            ir_result.data.push_back(adc_result[1]);
            ir_result.data.push_back(adc_result[0]);

            pub_ir_sensor_->publish(ir_result);

            auto batt_result = sensor_msgs::msg::BatteryState();
            batt_result.header.stamp = this->now();
            batt_result.voltage = (adc_result[4] / 4096.0) * 4.096 / (13.0 / 28.0);
            batt_result.temperature  = std::nan("");
            batt_result.current = std::nan("");
            batt_result.charge = std::nan("");
            batt_result.capacity = std::nan("");
            batt_result.design_capacity = 5.0;
            batt_result.percentage = std::nan("");
            batt_result.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            batt_result.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            batt_result.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
            batt_result.location = "base_link";
            batt_result.serial_number= "0";

            pub_batt_state_->publish(batt_result);
            RCLCPP_DEBUG(this->get_logger(), "%d %d %d %d %d", adc_result[0], adc_result[1], adc_result[2], adc_result[3], adc_result[4]);
        }

    private:
        int fd_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_us_sensor_;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_ir_sensor_;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_batt_state_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinkySensorADC>());

    rclcpp::shutdown();
    return 0;
}