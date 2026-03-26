#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math

from st3215 import ST3215

# Initialize the servo controller
servo = ST3215('/dev/ttyACM0')

TICKS_PER_REV = 4096
SQRT3 = 1.73205

class MotorOdom(Node):
    def __init__(self):
        super().__init__('motor_odom')

        # === PHYSICAL PARAMETERS ===
        self.wheel_radius = 0.05
        self.robot_radius = 0.15 # Distance from center of robot to the wheel
        
        self.servo_ids = [1, 2, 3] # 1: Front-Left, 2: Front-Right, 3: Back

        # === MOTOR INITIALIZATION ===
        for s_id in self.servo_ids:
            try:
                servo.SetMode(s_id, 1)
                servo.StartServo(s_id)
            except:
                pass

        # === STATE (3 Independent Wheels) ===
        self.prev_1 = None
        self.prev_2 = None
        self.prev_3 = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # === ROS INFRASTRUCTURE ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.033, self.update)

    # === TELEOP HANDLER (INVERSE KINEMATICS) ===
    def cmd_callback(self, msg):
        """Converts teleop commands into 3-wheel omni speeds"""
        vx = msg.linear.x  # Forward/Back
        vy = msg.linear.y  # Strafing (Left/Right)
        w  = msg.angular.z # Spinning

        # Omni-wheel matrix math
        # If your wheels spin backward, flip the + and - signs here!
        v1 = -(SQRT3 / 2.0) * vx + 0.5 * vy + self.robot_radius * w  # Front Left
        v2 =  (SQRT3 / 2.0) * vx + 0.5 * vy + self.robot_radius * w  # Front Right
        v3 =  0.0 * vx           - 1.0 * vy + self.robot_radius * w  # Back

        # Keep the multiplier so your q/z speed controls work!
        mult = 3000 
        
        try:
            servo.Rotate(1, int(v1 * mult))
            servo.Rotate(3, int(v2 * mult))
            servo.Rotate(2, int(v3 * mult))
        except Exception:
            pass

    # === READ ALL 3 MOTORS INDEPENDENTLY ===
    def read_motor_positions(self):
        try:
            p1 = servo.ReadPosition(1)
            p2 = servo.ReadPosition(2)
            p3 = servo.ReadPosition(3)
            if p1 is None or p2 is None or p3 is None:
                return self.prev_1, self.prev_2, self.prev_3
            return p1, p2, p3
        except Exception:
            return self.prev_1, self.prev_2, self.prev_3

    def compute_delta(self, curr, prev):
        delta = curr - prev
        if delta > TICKS_PER_REV / 2: delta -= TICKS_PER_REV
        elif delta < -TICKS_PER_REV / 2: delta += TICKS_PER_REV
        return delta

    # === ODOMETRY MAPPER (FORWARD KINEMATICS) ===
    def update(self):
        p1, p2, p3 = self.read_motor_positions()
        if p1 is None: return

        if self.prev_1 is None:
            self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3
            self.last_time = self.get_clock().now()
            return

        dist_per_tick = (2.0 * math.pi * self.wheel_radius) / TICKS_PER_REV
        d1 = self.compute_delta(p1, self.prev_1) * dist_per_tick
        d2 = self.compute_delta(p2, self.prev_2) * dist_per_tick
        d3 = self.compute_delta(p3, self.prev_3) * dist_per_tick
        
        self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3

        # 3-Wheel Omni Odometry Math
        # This will perfectly sync the map with your camera!
        dx_local = (d2 - d1) / SQRT3
        dy_local = (d1 + d2 - (2.0 * d3)) / 3.0
        dtheta   = (d1 + d2 + d3) / (3.0 * self.robot_radius)

        # Convert local movement to global map movement
        self.x += dx_local * math.cos(self.theta) - dy_local * math.sin(self.theta)
        self.y += dx_local * math.sin(self.theta) + dy_local * math.cos(self.theta)
        self.theta += dtheta

        current_time = self.get_clock().now()
        qw, qz = math.cos(self.theta / 2.0), math.sin(self.theta / 2.0)

        # Odom Msg
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id, odom.child_frame_id = "odom", "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(self.x), float(self.y)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = float(qz), float(qw)
        
        # Odom velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt > 0:
            odom.twist.twist.linear.x = float(dx_local / dt)
            odom.twist.twist.linear.y = float(dy_local / dt)
            odom.twist.twist.angular.z = float(dtheta / dt)

        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        # TF Broadcast
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id, t.child_frame_id = "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = float(self.x), float(self.y)
        t.transform.rotation.z, t.transform.rotation.w = float(qz), float(qw)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MotorOdom()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
