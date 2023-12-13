import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from custom_interfaces.srv import SetDistanceAngle 
import math

class OdomToPointServer(Node):

    def __init__(self):
        super().__init__('odom_to_point')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(PointStamped, '/destination', 10)
        self.srv = self.create_service(SetDistanceAngle, 'set_distance_angle', self.set_distance_angle_callback)
        self.last_pose = None
        self.distance = None
        self.angle = None

    def pose_callback(self, msg):
        # Store the latest pose but don't process it yet
        self.last_pose = msg

    def calculate_and_publish_new_point(self):
        if self.last_pose is None or self.distance is None or self.angle is None:
            return

        angle_rad = self.angle * (math.pi / 180)  # Convert angle to radians

        current_x = self.last_pose.pose.pose.position.x
        current_y = self.last_pose.pose.pose.position.y

        new_x = current_x + self.distance * math.cos(angle_rad)
        new_y = current_y + self.distance * math.sin(angle_rad)

        new_point = PointStamped()
        new_point.header.stamp = self.get_clock().now().to_msg()
        new_point.header.frame_id = self.last_pose.header.frame_id
        new_point.point.x = new_x
        new_point.point.y = new_y
        new_point.point.z = self.last_pose.pose.pose.position.z

        self.publisher.publish(new_point)
        self.get_logger().info(f'Old Point: [{current_x}, {current_y}], New Point: [{new_x}, {new_y}]')

    def set_distance_angle_callback(self, request, response):
        self.distance = request.distance
        self.angle = request.angle

        # Now calculate and publish the new point
        self.calculate_and_publish_new_point()

        response.success = True
        response.message = f"Distance set to {self.distance}, angle set to {self.angle}"
        return response

def main(args=None):
    rclpy.init(args=args)
    odom_to_point_publisher = OdomToPointServer()
    rclpy.spin(odom_to_point_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
