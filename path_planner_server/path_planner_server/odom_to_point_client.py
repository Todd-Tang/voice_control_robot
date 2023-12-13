import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SetDistanceAngle 

class OdomToPointClient(Node):

    def __init__(self):
        super().__init__('odom_to_point_client')
        self.client = self.create_client(SetDistanceAngle, 'set_distance_angle')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for service to become available...')
        self.req = SetDistanceAngle.Request()

    def send_request(self, distance, angle):
        self.req.distance = distance
        self.req.angle = angle
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    odom_to_point_client = OdomToPointClient()
    distance = float(input("Enter distance: "))
    angle = float(input("Enter angle: "))
    odom_to_point_client.send_request(distance, angle)

    while rclpy.ok():
        rclpy.spin_once(odom_to_point_client)
        if odom_to_point_client.future.done():
            try:
                response = odom_to_point_client.future.result()
            except Exception as e:
                odom_to_point_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                odom_to_point_client.get_logger().info(
                    'Result: %s' % (response.message))
            break

    odom_to_point_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
