#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class RoverNavigator(Node):
    def __init__(self):
        super().__init__('rover_navigator')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._action_client.wait_for_server()

    def send_target_pose(self, x, y, z=0.00, w=1.0):
      
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = float(z)

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = float(w)

        self.get_logger().info(f'Sending new goal: X={x}, Y={y}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        t1 = False

        self._send_goal_future.add_done_callback(t1= self.goal_response_callback)
        return t1

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server.')
            return False
        
        t1= False

        self.get_logger().info('Goal accepted! Navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(t1 = self.get_result_callback)
        return t1

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} meters')

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status code: {status}')
        return True


def main(args=None):
    rclpy.init(args=args)
    navigator = RoverNavigator()
    

    t1 = navigator.send_target_pose(1.75, -0.401)
    if(t1):
        navigator.send_target_pose(1.40 , -1.901)

    



    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down navigator node.')
        
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
