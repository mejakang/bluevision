import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from drone_interfaces.action import GoToPose
from rclpy.executors import MultiThreadedExecutor

class JoyCommandNode(Node):

    def __init__(self):
        super().__init__('joy_command_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.client = ActionClient(self, GoToPose, 'GoToPose')
        self.sent = False

    def joy_callback(self, msg):
        if msg.buttons[0] == 1 and not self.sent:
            self.get_logger().info("START")
            self.send_goal()
            self.sent = True

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Action Server Not Available")
            return

        goal_msg = GoToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 16.0
        pose.pose.position.y = 12.0
        pose.pose.position.z = 13.0
        goal_msg.target_pose = pose

        self.get_logger().info("Sending Goal to Server")
        send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal Rejected')
            return

        self.get_logger().info('Goal Accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.current_status}')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Result: {result.message}')
        else:
            self.get_logger().warn(f'Action Failed: {result.message}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyCommandNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

