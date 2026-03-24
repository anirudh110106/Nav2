#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

WHEEL_TOPIC = "/base_velocity_controller/commands"

class WheelTester(Node):

    def __init__(self):
        super().__init__("wheel_tester")
        self.pub = self.create_publisher(Float64MultiArray, WHEEL_TOPIC, 10)
        self.get_logger().info("Wheel tester ready")

    def omni_kinematics(self, Vx, Vy, W):
        R = 1.0
        theta1 = 0
        theta2 = 2 * math.pi / 3
        theta3 = 4 * math.pi / 3

        w1 = -math.sin(theta1) * Vx + math.cos(theta1) * Vy + R * W
        w2 = -math.sin(theta2) * Vx + math.cos(theta2) * Vy + R * W
        w3 = -math.sin(theta3) * Vx + math.cos(theta3) * Vy + R * W

        return [-w1, -w2, -w3]

    def publish_wheels(self, Vx, Vy, W):
        # Same axis correction as follow.py
        Vx_fixed = Vy
        Vy_fixed = -Vx
        speeds = self.omni_kinematics(Vx_fixed, Vy_fixed, W)
        msg = Float64MultiArray()
        msg.data = speeds
        self.pub.publish(msg)
        self.get_logger().info(f"Wheels: {[f'{s:.2f}' for s in speeds]}")

    def stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.pub.publish(msg)
        self.get_logger().info("Stopped")

    # ── the three movement functions ──────────────────────────────────────

    def move_forward(self, speed=2.0, duration=2.0):
        self.get_logger().info(f"Moving forward | speed={speed} | duration={duration}s")
        end = time.time() + duration
        while time.time() < end:
            self.publish_wheels(Vx=0.0, Vy=-speed, W=0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def turn_left(self, speed=2.0, duration=2.0):
        # Positive W = counter-clockwise = turn left
        self.get_logger().info(f"Turning left | speed={speed} | duration={duration}s")
        end = time.time() + duration
        while time.time() < end:
            self.publish_wheels(Vx=0.0, Vy=0.0, W=speed)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def turn_right(self, speed=2.0, duration=2.0):
        # Negative W = clockwise = turn right
        self.get_logger().info(f"Turning right | speed={speed} | duration={duration}s")
        end = time.time() + duration
        while time.time() < end:
            self.publish_wheels(Vx=0.0, Vy=0.0, W=-speed)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()


def main():
    rclpy.init()
    node = WheelTester()

    # ── test sequence — swap these out as needed ──
    node.move_forward(speed=2.0, duration=10.0)
    time.sleep(0.5)
    node.turn_left(speed=2.0, duration=1.5)
    time.sleep(0.5)
    node.turn_right(speed=2.0, duration=1.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()