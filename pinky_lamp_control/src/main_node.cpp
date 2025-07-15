#include "rclcpp/rclcpp.hpp"

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
            .strip_type = WS2811_STRIP_GRB,
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
            if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
            {
                assert(false);
            }

            matrix_fill(0xff000000);
            matrix_render();
            ws2811_render(&ledstring);
        }
        ~PinkyLampControl() {}

    private:
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

    private:
        ws2811_led_t* matrix_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinkyLampControl>());

    rclcpp::shutdown();
    return 0;
}