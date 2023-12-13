import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from custom_interfaces.action import CalculatePoint 
import math

class OdomToPointActionServer(Node):

    def __init__(self):
        super().__init__('odom_to_point_action_server')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self._action_server = ActionServer(
            self,
            CalculatePoint,
            'calculate_point',
            self.execute_callback)
        self.last_pose = None

    def pose_callback(self, msg):
        self.last_pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        distance = goal_handle.request.distance
        angle = goal_handle.request.angle
        new_point = self.calculate_new_point(distance, angle)

        if new_point is not None:
            goal_handle.succeed()
            result = CalculatePoint.Result()
            result.x = new_point.point.x
            result.y = new_point.point.y
            result.z = new_point.point.z
            return result

    def calculate_new_point(self, distance, angle):
        if self.last_pose is None:
            return None

        angle_rad = angle * (math.pi / 180)  # Convert angle to radians

        current_x = self.last_pose.pose.pose.position.x
        current_y = self.last_pose.pose.pose.position.y

        new_x = current_x + distance * math.cos(angle_rad)
        new_y = current_y + distance * math.sin(angle_rad)

        new_point = PointStamped()
        new_point.header.stamp = self.get_clock().now().to_msg()
        new_point.header.frame_id = self.last_pose.header.frame_id
        new_point.point.x = new_x
        new_point.point.y = new_y
        new_point.point.z = self.last_pose.pose.pose.position.z

        return new_point

def main(args=None):
    rclpy.init(args=args)
    action_server = OdomToPointActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
