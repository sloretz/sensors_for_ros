#/usr/bin/env python3

import random

import rclpy
import rclpy.node
from std_msgs.msg import ColorRGBA

import time

def main():
    print('initializing rclpy')
    rclpy.init()

    node = rclpy.node.Node('color_changer')
    pub = node.create_publisher(ColorRGBA, 'color', 1)

    clock = node.get_clock()
    msg = ColorRGBA()
    msg.r = 1.0
    msg.g = 1.0
    msg.b = 1.0
    msg.a = 1.0

    print('created node')

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        color_to_change = random.choice(('r', 'g', 'b'))
        change_amount = 0.01
        if 'r' == color_to_change:
            msg.r += change_amount
        elif 'g' == color_to_change:
            msg.g += change_amount
        elif 'b' == color_to_change:
            msg.b += change_amount

        if msg.r > 1.0:
            msg.r = 0.0;
        if msg.g > 1.0:
            msg.g = 0.0;
        if msg.b > 1.0:
            msg.b = 0.0;
        pub.publish(msg)
        time.sleep(0.01)


if __name__ == '__main__':
    main()
