import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
from drone_interfaces.action import GoToPose
import time
import math


class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')

        # Action Server
        self._action_server = ActionServer(
            self,
            GoToPose,
            'GoToPose',
            self.execute_callback
        )

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Subscriber for current position
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.position_callback,
            10
        )
        self.current_position = None

    def position_callback(self, msg):
        self.current_position = msg

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info(f"[VehicleCommand] Sent: {command}")

    async def execute_callback(self, goal_handle):
        self.get_logger().info("[Action] Goal received.")

        pose = goal_handle.request.target_pose

        # Wait for current position
        while self.current_position is None and rclpy.ok():
            self.get_logger().info("[Position] Waiting for current position...")
            time.sleep(0.1)

        # ENU → NED 변환
        target_x = pose.pose.position.y  # ENU.y → NED.x
        target_y = pose.pose.position.x  # ENU.x → NED.y
        target_z = -pose.pose.position.z  # ENU.z → NED.z

        # 준비된 TrajectorySetpoint
        traj = TrajectorySetpoint()
        traj.position = [target_x, target_y, target_z]
        traj.yaw = 0.0

        # OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False

        # Pre-flight setpoint publish (필수)
        for _ in range(10):
            timestamp = self.get_clock().now().nanoseconds // 1000
            offboard_msg.timestamp = timestamp
            traj.timestamp = timestamp
            self.offboard_pub.publish(offboard_msg)
            self.trajectory_pub.publish(traj)
            time.sleep(0.1)

        # Arm
        self.send_vehicle_command(command=400, param1=1.0)
        time.sleep(0.5)

        # Offboard Mode On
        self.send_vehicle_command(command=176, param1=1.0)
        time.sleep(0.5)

        # Action Feedback: Moving
        feedback_msg = GoToPose.Feedback()
        feedback_msg.current_status = 'Moving'
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info(f"[Action] Moving to: x={target_x}, y={target_y}, z={target_z}")

        # 목표점까지 주기적 setpoint publish + 위치 판별 루프
        tolerance = 0.2  # 도착 오차(m)

        while rclpy.ok():
            timestamp = self.get_clock().now().nanoseconds // 1000
            offboard_msg.timestamp = timestamp
            traj.timestamp = timestamp
            self.offboard_pub.publish(offboard_msg)
            self.trajectory_pub.publish(traj)

            if self.current_position is not None:
                dx = self.current_position.x - target_x
                dy = self.current_position.y - target_y
                dz = self.current_position.z - target_z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                self.get_logger().info(f"[Distance] Current dist: {distance:.2f} m")

                if distance < tolerance:
                    break

            time.sleep(0.1)

        # 도착!
        feedback_msg.current_status = 'Arrived'
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info("[Action] Arrived at Target!")

        result = GoToPose.Result()
        result.success = True
        result.message = "Arrived at Destination"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

