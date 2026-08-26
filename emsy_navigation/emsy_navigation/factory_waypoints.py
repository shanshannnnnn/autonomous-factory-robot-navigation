import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class FactoryWaypoints(Node):
    def __init__(self):
        super().__init__('factory_waypoints')
        self.client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )
        self.waypoints = [
            (0.68, 4.14, 'Workbench 1'),
            (2.34, 1.33, 'Workbench 2'),
            (4.18, -3.60, 'Charging Station')
        ]
        self.current_waypoint = 0
        self.get_logger().info('Waiting for Nav2...')
        self.client.wait_for_server()
        self.get_logger().info('Nav2 connected!')
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            return

        x, y, name = self.waypoints[self.current_waypoint]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Going to {name}: x={x}, y={y}'
        )

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        x, y, name = self.waypoints[self.current_waypoint]

        if status == 4:
            self.get_logger().info(f'{name} reached!')
            self.current_waypoint += 1
            self.send_next_goal()
        else:
            self.get_logger().error(
                f'Failed to reach {name}. Status={status}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FactoryWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()