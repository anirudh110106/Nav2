#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')
TICKS_PER_REV = 4096
SQRT3 = 1.73205

class MotorOdom(Node):
    def __init__(self):
        super().__init__('motor_odom')

        self.wheel_radius = 0.05
        self.robot_radius = 0.15

        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        # Encoder tracking
        self.prev_1 = self.prev_2 = self.prev_3 = None
        self.acc1 = self.acc2 = self.acc3 = 0
        self.prev_acc1 = self.prev_acc2 = self.prev_acc3 = 0

        # Motion smoothing
        self.prev_dx = 0.0
        self.prev_dy = 0.0

        # Offsets
        self.drive_heading_offset = 120.0
        self.odom_heading_offset = 117.0   # tune this slightly

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.timer = self.create_timer(0.033, self.update)

    # ---------------- DRIVE ----------------
    def cmd_callback(self, msg):
        cmd_x, cmd_y, w = msg.linear.x, msg.linear.y, msg.angular.z

        rad = math.radians(self.drive_heading_offset)

        vx = cmd_x * math.cos(rad) - cmd_y * math.sin(rad)
        vy = cmd_x * math.sin(rad) + cmd_y * math.cos(rad)

        v1 = -(SQRT3/2.0)*vx + 0.5*vy + self.robot_radius*w
        v2 = (SQRT3/2.0)*vx + 0.5*vy + self.robot_radius*w
        v3 = -1.0*vy + self.robot_radius*w

        try:
            for i, v in enumerate([v1, v2, v3], 1):
                servo.Rotate(i, int(v * 3000))
        except:
            pass

    # ---------------- ENCODER UNWRAP ----------------
    def unwrap(self, curr, prev, acc):
        diff = curr - prev

        if diff > 2048:
            diff -= 4096
        elif diff < -2048:
            diff += 4096

        # STRICT filter (major fix)
        if abs(diff) > 300:
            return acc, prev

        return acc + diff, curr

    # ---------------- UPDATE ----------------
    def update(self):
        try:
            p1 = servo.ReadPosition(1)
            p2 = servo.ReadPosition(2)
            p3 = servo.ReadPosition(3)
        except:
            return

        if None in (p1, p2, p3):
            return

        if self.prev_1 is None:
            self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3
            return

        # unwrap encoders
        self.acc1, self.prev_1 = self.unwrap(p1, self.prev_1, self.acc1)
        self.acc2, self.prev_2 = self.unwrap(p2, self.prev_2, self.acc2)
        self.acc3, self.prev_3 = self.unwrap(p3, self.prev_3, self.acc3)

        # delta ticks
        d1 = self.acc1 - self.prev_acc1
        d2 = self.acc2 - self.prev_acc2
        d3 = self.acc3 - self.prev_acc3

        self.prev_acc1 = self.acc1
        self.prev_acc2 = self.acc2
        self.prev_acc3 = self.acc3

        # convert to meters
        scale = (2.0 * math.pi * self.wheel_radius / TICKS_PER_REV)
        d1 *= scale
        d2 *= scale
        d3 *= scale

        # REJECT IMPOSSIBLE MOTION (critical fix)
        if abs(d1) > 0.05 or abs(d2) > 0.05 or abs(d3) > 0.05:
            return

        # kinematics
        raw_dx = (d2 - d1) / SQRT3
        raw_dy = (d1 + d2 - 2.0*d3) / 3.0

        # rotate frame
        rad = math.radians(-self.odom_heading_offset)
        dx = raw_dx * math.cos(rad) - raw_dy * math.sin(rad)
        dy = raw_dx * math.sin(rad) + raw_dy * math.cos(rad)

        # SMOOTHING (removes spikes)
        alpha = 0.6
        dx = alpha * dx + (1 - alpha) * self.prev_dx
        dy = alpha * dy + (1 - alpha) * self.prev_dy
        self.prev_dx = dx
        self.prev_dy = dy

        # angular motion
        ANGULAR_SCALE = 0.6
        dth = ANGULAR_SCALE * (d1 + d2 + d3) / (3.0 * self.robot_radius)

        # midpoint integration
        avg_theta = self.theta + dth / 2.0

        self.x += dx * math.cos(avg_theta) - dy * math.sin(avg_theta)
        self.y += dx * math.sin(avg_theta) + dy * math.cos(avg_theta)
        self.theta += dth

        # ---------------- TF ----------------
        now = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = "odom"
        t1.child_frame_id = "base_footprint"
        t1.transform.translation.x = float(self.x)
        t1.transform.translation.y = float(self.y)
        t1.transform.rotation.z = qz
        t1.transform.rotation.w = qw

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "base_footprint"
        t2.child_frame_id = "base_link"
        t2.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t1, t2])

        # ---------------- ODOM ----------------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = MotorOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()