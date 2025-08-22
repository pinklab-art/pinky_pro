import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from PIL import Image, ImageSequence
import os
from pinky_interfaces.srv import Emotion

from .pinky_lcd import LCD

class PinkyEmotion(Node):
    def __init__(self):
        super().__init__('pinky_emotion')

        self.emotion_path = os.path.join(get_package_share_directory('pinky_emotion'), 'emotion')
        self.emotion_service = self.create_service(Emotion, 'set_emotion', self.lcd_callback)

        self.lcd = LCD()
        self.get_logger().info(f"Pinky's emotion server is ready!!")

    def lcd_callback(self, request, response):
        emo = request.emotion
        self.get_logger().info(f"Pinky's emotion set to {emo}")
        response.response = f"Pinky's emotion set to {emo}"
        
        if emo == "hello":
            self.play_gif(self.emotion_path + "/hello.gif")
        
        elif emo == "basic":
            self.play_gif(self.emotion_path + "/basic.gif")
        
        elif emo == "angry":
            self.play_gif(self.emotion_path + "/angry.gif")
        
        elif emo == "bored":
            self.play_gif(self.emotion_path + "/bored.gif")

        elif emo == "fun":
            self.play_gif(self.emotion_path + "/fun.gif")
            
        elif emo == "happy":
            self.play_gif(self.emotion_path + "/happy.gif")

        elif emo == "interest":
            self.play_gif(self.emotion_path + "/interest.gif")

        elif emo == "sad":
            self.play_gif(self.emotion_path + "/sad.gif")

        else:
            response.response = "wrong command"
            self.get_logger().info("wrong command")

        self.lcd.clear()

        return response

    def play_gif(self, path):
        img = Image.open(path)
        for i, frame in enumerate(ImageSequence.Iterator(img)):
            if i % 2 == 0:
                self.lcd.img_show(frame)

        
def main(args=None):
    rclpy.init(args=args)
    pinky_lcd_node = PinkyEmotion()
     
    try:
        rclpy.spin(pinky_lcd_node)
    except KeyboardInterrupt:
        pass
    finally:
        pinky_lcd_node.lcd.clear()
        pinky_lcd_node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()