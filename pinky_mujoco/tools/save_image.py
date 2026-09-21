"""Save one frame of /camera/image_raw to a PNG (default ./camera_frame.png)."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PILImage


def main():
    output = sys.argv[1] if len(sys.argv) > 1 else 'camera_frame.png'
    rclpy.init()
    node = Node('pinky_save_image')
    frames = []
    node.create_subscription(Image, '/camera/image_raw', frames.append, qos_profile_sensor_data)
    while not frames and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    msg = frames[0]
    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    PILImage.fromarray(array).save(output)
    print(f'{output}: {msg.width}x{msg.height} {msg.encoding} frame_id={msg.header.frame_id}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
