import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math
import time



from st3215 import ST3215
import time

servo = ST3215('/dev/ttyACM0')

TICKS_PER_REV = 4096

class MotorOdom(Node):

    def __init__(self):
        super().__init__('motor_odom')

        # === PARAMETERS (CHANGE THESE) ===
        self.wheel_radius = 0.05      # meters (example: 5 cm)
        self.wheel_base = 0.30        # distance between wheels (meters)

        # === STATE ===
        self.prev_left = None
        self.prev_right = None

        self.left_ticks_total = 0
        self.right_ticks_total = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # === ROS PUB ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.update)

    # === REPLACE THIS WITH YOUR MOTOR READ FUNCTION ===
    def read_motor_positions(self):
        try:
            # === MOTOR IDS ===
            LEFT_IDS = [1, 2]
            RIGHT_ID = 3

            # --- Read left motors ---
            left_vals = []
            for mid in LEFT_IDS:
                pos = servo.ReadPosition(mid)
                if pos is not None:
                    left_vals.append(pos)

            # --- Read right motor ---
            right_pos = servo.ReadPosition(RIGHT_ID)

            # --- Safety check ---
            if len(left_vals) == 0 or right_pos is None:
                self.get_logger().warn("Motor read failed")
                return self.prev_left, self.prev_right

            # --- Average left side ---
            left_pos = sum(left_vals) / len(left_vals)

            return left_pos, right_pos

        except Exception as e:
            self.get_logger().error(f"Read error: {e}")
            return self.prev_left, self.prev_right
        
            

    # === HANDLE WRAP-AROUND ===
    def compute_delta(self, curr, prev):
        delta = curr - prev

        if delta > TICKS_PER_REV / 2:
            delta -= TICKS_PER_REV
        elif delta < -TICKS_PER_REV / 2:
            delta += TICKS_PER_REV

        return delta

    def update(self):
        left_pos, right_pos = self.read_motor_positions()

        if self.prev_left is None:
            self.prev_left = left_pos
            self.prev_right = right_pos
            return

        # === DELTA TICKS ===
        delta_left = self.compute_delta(left_pos, self.prev_left)
        delta_right = self.compute_delta(right_pos, self.prev_right)

        self.prev_left = left_pos
        self.prev_right = right_pos

        # === CONVERT TO DISTANCE ===
        dist_per_tick = (2 * math.pi * self.wheel_radius) / TICKS_PER_REV

        d_left = delta_left * dist_per_tick
        d_right = delta_right * dist_per_tick

        # === DIFFERENTIAL DRIVE ===
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # === UPDATE POSE ===
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        # === TIME ===
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        vx = d_center / dt if dt > 0 else 0.0
        vth = d_theta / dt if dt > 0 else 0.0

        # === ODOM MESSAGE ===
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # === TF BROADCAST ===
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MotorOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()