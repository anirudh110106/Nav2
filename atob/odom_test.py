#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')

TICKS_PER_REV = 4096

class MotorOdom(Node):
    def __init__(self):
        super().__init__('motor_odom')

        # === PARAMETERS ===
        self.wheel_radius = 0.05
        self.wheel_base = 0.30

        # === STATE ===
        self.prev_left = None
        self.prev_right = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # === ROS PUB ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.update)

    # === MOTOR READ ===
    def read_motor_positions(self):
        try:
            LEFT_IDS = [1, 2]
            RIGHT_ID = 3

            left_vals = []
            for mid in LEFT_IDS:
                pos = servo.ReadPosition(mid)
                if pos is not None:
                    left_vals.append(pos)

            right_pos = servo.ReadPosition(RIGHT_ID)

            if len(left_vals) == 0 or right_pos is None:
                # Suppress warning to avoid spamming terminal, just return previous
                return self.prev_left, self.prev_right

            left_pos = sum(left_vals) / len(left_vals)
            return left_pos, right_pos

        except Exception as e:
            self.get_logger().error(f"Read error: {e}")
            return self.prev_left, self.prev_right

    # === WRAP HANDLING ===
    def compute_delta(self, curr, prev):
        delta = curr - prev
        if delta > TICKS_PER_REV / 2:
            delta -= TICKS_PER_REV
        elif delta < -TICKS_PER_REV / 2:
            delta += TICKS_PER_REV
        return delta

    def update(self):
        left_pos, right_pos = self.read_motor_positions()

        if left_pos is None or right_pos is None:
            return

        # Initialize previous positions on the first run
        if self.prev_left is None:
            self.prev_left = left_pos
            self.prev_right = right_pos
            self.last_time = self.get_clock().now() # Reset time to prevent huge first dt
            return

        # === DELTA TICKS ===
        delta_left = self.compute_delta(left_pos, self.prev_left)
        delta_right = self.compute_delta(right_pos, self.prev_right)

        self.prev_left = left_pos
        self.prev_right = right_pos

        # === DISTANCE ===
        dist_per_tick = (2.0 * math.pi * self.wheel_radius) / TICKS_PER_REV

        d_left = delta_left * dist_per_tick
        d_right = delta_right * dist_per_tick

        # === DIFFERENTIAL DRIVE KINEMATICS ===
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # === UPDATE POSE ===
        # We integrate the pose continuously without dropping "small" movements
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        # === TIME & VELOCITY ===
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        vx = d_center / dt if dt > 0 else 0.0
        vth = d_theta / dt if dt > 0 else 0.0

        # === NATIVE PYTHON QUATERNION (Replaces scipy) ===
        # Scipy outputs numpy float64 which crashes ROS 2 serialization
        qw = math.cos(self.theta / 2.0)
        qz = math.sin(self.theta / 2.0)

        # === ODOM MESSAGE ===
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(vth)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom)

        # === TF BROADCAST ===
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MotorOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
