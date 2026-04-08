#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class RoverNavigator(Node):
    def __init__(self):
        super().__init__('rover_navigator')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Wait for the action server to be ready once during startup
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server found! Ready to receive goals.')

    def send_target_pose(self, x, y, z=0.0, w=1.0):
        """
        Call this function anytime to send a new destination to the rover.
        If the rover is currently moving, Nav2 will preempt the old goal.
        """
        goal_msg = NavigateToPose.Goal()

        # 1. Set Header
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 2. Set Position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = float(z)

        # 3. Set Orientation (using Quaternions)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = float(w)

        self.get_logger().info(f'Sending new goal: X={x}, Y={y}')
        
        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server.')
            return

        self.get_logger().info('Goal accepted! Navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Logs distance remaining. Comment this out if it spams your terminal too much."""
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} meters')

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status code: {status}')


def main(args=None):
    rclpy.init(args=args)
    navigator = RoverNavigator()
    
    # --- Send your initial goal ---
    navigator.send_target_pose(x=1.75, y=-0.401, z=0.0001, w=1.0)
    
    # --- Example: Sending a new goal dynamically ---
    # If you wanted to interrupt the rover and send it somewhere else 
    # after 5 seconds, you could trigger the function via a timer like this:
    #
    # timer = navigator.create_timer(5.0, lambda: navigator.send_target_pose(x=0.0, y=0.0, w=1.0))

    try:
        # Keep the node alive to listen for callbacks and new commands
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down navigator node.')
        
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
