#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "pinky_interfaces/srv/set_lamp.hpp"

#include "ws2811/clk.h"
#include "ws2811/gpio.h"
#include "ws2811/dma.h"
#include "ws2811/pwm.h"
#include "ws2811/ws2811.h"


ws2811_t ledstring =
{
    .freq = WS2811_TARGET_FREQ,
    .dmanum = 10,
    .channel =
    {
        [0] =
        {
            .gpionum = 19,
            .invert = 0,
            .count = 8,
            .strip_type = WS2811_STRIP_GBR,
            .brightness = 255,
        },
    },
};

class PinkyLampControl : public rclcpp::Node
{
    public:
        PinkyLampControl() : Node("pinky_lamp_control")
        {
            matrix_ = (ws2811_led_t*)malloc(sizeof(ws2811_led_t) * 8);

            ws2811_return_t ret;
            // ws2811_init(&ledstring);
            if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Error ws2811_init. %d", ret);
                assert(false);
            }

            matrix_fill(0xff000000);
            matrix_render();
            ws2811_render(&ledstring);

            goal_r_ = 255;
            goal_g_ = 255;
            goal_b_ = 255;
            current_mode_ = 3;
            current_time_ = 1000;
            time_index_ = 0;
            current_dir_ = 0;
            target_color_ = 0xff000000;

            auto [h, s, v] = RGBtoHSV(goal_r_, goal_g_, goal_b_);
            goal_h_ = h;
            goal_s_ = s;
            goal_v_ = v;

            srv_set_lamp_ = this->create_service<pinky_interfaces::srv::SetLamp>("set_lamp",
                std::bind(&PinkyLampControl::callback_set_lamp, this, std::placeholders::_1, std::placeholders::_2)
            );

            auto period = std::chrono::duration<double>(1.0 / 100.0);
            timer_ = this->create_wall_timer(period, std::bind(&PinkyLampControl::timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "%s initialized...", this->get_name());
        }
        ~PinkyLampControl()
        {
            // Turn off all lamps.
            matrix_fill(0xff000000);
            matrix_render();
            ws2811_render(&ledstring);
        }

    private:
        void callback_set_lamp(const std::shared_ptr<pinky_interfaces::srv::SetLamp::Request> req,
                                        std::shared_ptr<pinky_interfaces::srv::SetLamp::Response> res)
        {
            goal_r_ = (uint8_t)(req->color.r * 255);
            goal_g_ = (uint8_t)(req->color.g * 255);
            goal_b_ = (uint8_t)(req->color.b * 255);

            current_mode_ = req->mode;
            current_time_ = req->time;

            time_index_ = 0;
            current_dir_ = 0;

            if(req->mode == 3)
            {
                auto [h, s, v] = RGBtoHSV(goal_r_, goal_g_, goal_b_);
                goal_h_ = h;
                goal_s_ = s;
                goal_v_ = v;
            }

            RCLCPP_INFO(this->get_logger(), "Set lamp to Color [%d %d %d], Mode [%d] and Time [%d].",
                                            goal_r_, goal_g_, goal_b_, current_mode_, current_time_);

            res->result = true;
        }

        void timer_callback()
        {
            uint32_t initial_color = 0xff000000;

            if(current_mode_ == 0) // turn off
            {
                target_color_ = initial_color;
            }
            else if(current_mode_ == 1) // turn on
            {
                target_color_ = initial_color | (uint32_t)(goal_r_ << 16) | (uint32_t)(goal_g_ << 8) | (uint32_t)(goal_b_);
            }
            else if(current_mode_ == 2) // blink
            {
                time_index_++;
                if(((time_index_ * 10) > current_time_) && current_dir_ == 0)
                {
                    target_color_ = initial_color | (uint32_t)(goal_r_ << 16) | (uint32_t)(goal_g_ << 8) | (uint32_t)(goal_b_);
                    time_index_ = 0;
                    current_dir_ = 1;
                }
                if(((time_index_ * 10) > current_time_) && current_dir_ == 1)
                {
                    target_color_ = 0xff000000;
                    time_index_ = 0;
                    current_dir_ = 0;
                }
            }
            else if(current_mode_ == 3) // dimming
            {
                if(current_dir_ == 0)
                    time_index_ += ((goal_v_ * 100) / (current_time_ / 10.0));
                else
                    time_index_ -= ((goal_v_ * 100) / (current_time_ / 10.0));

                if(time_index_ < 0)
                {
                    time_index_ = 0;
                    current_dir_ = 0;
                }
                else if(time_index_ > (goal_v_ * 100))
                {
                    time_index_ = (uint16_t)(goal_v_ * 100);
                    current_dir_ = 1;
                }

                auto [r, g, b] = HSVtoRGB(goal_h_, goal_s_, time_index_ / 100.0);
                target_color_ = initial_color | (uint32_t)(r << 16) | (uint32_t)(g << 8) | (uint32_t)(b);
            }

            matrix_fill(target_color_);
            matrix_render();
            ws2811_render(&ledstring);
        }

        void matrix_fill(ws2811_led_t color)
        {
            int x;
            for (x = 0; x < ledstring.channel[0].count; x++)
            {
                matrix_[x] = color;
            }
        }

        void matrix_render()
        {
            int x;
            for (x = 0; x < ledstring.channel[0].count; x++)
            {
                ledstring.channel[0].leds[x] = matrix_[x];
            }
        }

        std::tuple<int, int, int> HSVtoRGB(double h, double s, double v)
        {
            double c = v * s; // chroma
            double x = c * (1 - std::fabs(fmod(h / 60.0, 2) - 1));
            double m = v - c;

            double r_, g_, b_;

            if (h < 60)       { r_ = c; g_ = x; b_ = 0; }
            else if (h < 120) { r_ = x; g_ = c; b_ = 0; }
            else if (h < 180) { r_ = 0; g_ = c; b_ = x; }
            else if (h < 240) { r_ = 0; g_ = x; b_ = c; }
            else if (h < 300) { r_ = x; g_ = 0; b_ = c; }
            else              { r_ = c; g_ = 0; b_ = x; }

            uint8_t r = static_cast<uint8_t>((r_ + m) * 255);
            uint8_t g = static_cast<uint8_t>((g_ + m) * 255);
            uint8_t b = static_cast<uint8_t>((b_ + m) * 255);

            return {r, g, b};
        }

        std::tuple<double, double, double> RGBtoHSV(uint8_t r, uint8_t g, uint8_t b)
        {
            double r_ = r / 255.0f;
            double g_ = g / 255.0f;
            double b_ = b / 255.0f;

            double maxVal = std::max({r_, g_, b_});
            double minVal = std::min({r_, g_, b_});
            double delta = maxVal - minVal;

            double h = 0.0f;
            if (delta != 0.0f) {
                if (maxVal == r_) {
                    h = 60.0f * fmod(((g_ - b_) / delta), 6.0f);
                } else if (maxVal == g_) {
                    h = 60.0f * (((b_ - r_) / delta) + 2);
                } else {
                    h = 60.0f * (((r_ - g_) / delta) + 4);
                }
            }

            if (h < 0.0f)
                h += 360.0f;

            double s = (maxVal == 0) ? 0 : delta / maxVal;
            double v = maxVal;

            return {h, s, v};
        }

    private:
        ws2811_led_t* matrix_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Service<pinky_interfaces::srv::SetLamp>::SharedPtr srv_set_lamp_;

        uint8_t goal_r_;
        uint8_t goal_g_;
        uint8_t goal_b_;

        double goal_h_;
        double goal_s_;
        double goal_v_;

        uint8_t current_mode_;
        uint16_t current_time_;
        int16_t time_index_;
        uint8_t current_dir_;
        uint32_t target_color_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinkyLampControl>());

    rclcpp::shutdown();
    return 0;
}
