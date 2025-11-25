import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math
import time

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Parameters (adjust for your bot)
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_base', 0.16)     # distance between wheels (m)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.ser = serial.Serial(port, baud, timeout=1)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_L = 0
        self.last_R = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def update(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
            # Expected format: L:<count>,R:<count>,dt_ms:<interval>
            parts = dict(p.split(':') for p in line.split(','))
            L = int(parts['L'])
            R = int(parts['R'])
            dt = int(parts['dt_ms']) / 1000.0
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")
            return

        # Differences
        dL = (L - self.last_L)
        dR = (R - self.last_R)
        self.last_L, self.last_R = L, R

        # Convert ticks to distance
        # NOTE: adjust "ticks_per_rev"
        ticks_per_rev = 1440
        dist_per_tick = (2 * math.pi * self.get_parameter('wheel_radius').value) / ticks_per_rev
        dL_m = dL * dist_per_tick
        dR_m = dR * dist_per_tick

        # Differential drive kinematics
        d_center = (dL_m + dR_m) / 2.0
        d_theta = (dR_m - dL_m) / self.get_parameter('wheel_base').value

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        vx = d_center / dt
        vth = d_theta / dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.theta/2.0)
        qw = math.cos(self.theta/2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

