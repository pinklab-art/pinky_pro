import os
import sys
import time
import logging
import spidev as SPI

import rclpy
from rclpy.node import Node

from . import LCD_2inch4
from PIL import Image, ImageDraw, ImageFont

RST = 27
DC = 25
BL = 18
bus = 0
device = 0
logging.basicConfig(level=logging.DEBUG)

class PinkyScreenControlNode(Node):
    def __init__(self):
        super().__init__('pinky_screen_control_node')

        self.disp = LCD_2inch4.LCD_2inch4()
        self.disp.Init()
        self.disp.clear()

        image1 = Image.new("RGB", (self.disp.height, self.disp.width), "BLUE")
        draw = ImageDraw.Draw(image1)

        # Font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 25)
        Font1 = ImageFont.truetype("Lato-Black.ttf", 36)
        draw.text((62, 100), 'Hello, Pinky', fill = "PINK", font=Font1)
        image1 = image1.transpose(Image.FLIP_LEFT_RIGHT).transpose(Image.ROTATE_270)

        self.disp.ShowImage(image1)


def main(args=None):
    rclpy.init(args=args)

    pinky_screen_control_node = PinkyScreenControlNode()

    rclpy.spin(pinky_screen_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    screen_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()