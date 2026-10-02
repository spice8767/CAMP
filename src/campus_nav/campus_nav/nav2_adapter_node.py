import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from std_msgs.msg import String

class Nav2AdapterNode(Node):
    def __init__(self):
        super().__init__('nav2_adapter_node')
        
        # Subscribe to the exact topic the mission controller already publishes
        self.path_sub = self.create_subscription(
            Path, '/campus/target_path', self.path_callback, 10)
            
        # We need to tell the mission controller our status so it knows when we arrive
        self.status_pub = self.create_publisher(String, '/campus/pursuit_status', 10)
        
        # Action client to talk to Nav2's controller_server
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info("Nav2 Adapter Node ready. Waiting for /campus/target_path...")

    def path_callback(self, msg: Path):
        self.get_logger().info(f"Received path with {len(msg.poses)} waypoints! Sending to Nav2...")
        
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 controller_server action server not available!")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = 'FollowPath'

        # Send the action goal
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Tell the mission controller we started!
        status_msg = String()
        status_msg.data = "[Nav2] Following: 0% | Driving..."
        self.status_pub.publish(status_msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the path!")
            return
        self.get_logger().info("Nav2 accepted the path! Driving...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # In rclpy.action.GoalStatus, 4 is SUCCEEDED, 5 is CANCELED, 6 is ABORTED
        if status == 4:
            self.get_logger().info("Nav2 reached the destination successfully!")
            status_msg = String()
            status_msg.data = "ARRIVED"
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f"Nav2 failed to reach destination! Status code: {status}")
            # Do not publish ARRIVED so mission doesn't falsely complete!

def main(args=None):
    rclpy.init(args=args)
    node = Nav2AdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
