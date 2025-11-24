#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(String, '/teleop/cmd', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        print('=== PX4 Keyboard Control ===')
        print('w/s: Forward/Back | a/d: Left/Right')
        print('q/e: Up/Down | Space: Hover')
        print('t: Takeoff | l: Land | k: Disarm')
        print('Ctrl+C: Exit')
        print('----------------------------')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                
                if key == '\x03':  # Ctrl+C
                    break
                    
                msg = String()
                if key == ' ':
                    msg.data = 'space'
                else:
                    msg.data = key
                    
                self.pub.publish(msg)
                
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
