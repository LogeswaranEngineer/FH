# File: cmdvel_to_serial.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Update if needed

    def listener_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0.0:
            self.send_command('F')
        elif linear < 0.0:
            self.send_command('B')
        elif angular > 0.0:
            self.send_command('L')
        elif angular < 0.0:
            self.send_command('R')
        else:
            self.send_command('S')

    def send_command(self, cmd_char):
        self.serial_port.write(cmd_char.encode())

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
