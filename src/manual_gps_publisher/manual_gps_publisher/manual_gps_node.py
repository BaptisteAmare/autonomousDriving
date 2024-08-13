import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class ManualGPSNode(Node):
    def __init__(self):
        super().__init__('manual_gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_gps)
        self.latitude = None
        self.longitude = None

    def publish_gps(self):
        if self.latitude is not None and self.longitude is not None:
            msg = NavSatFix()
            msg.latitude = self.latitude
            msg.longitude = self.longitude
            self.publisher_.publish(msg)

    def set_gps(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

def main(args=None):
    rclpy.init(args=args)
    manual_gps_node = ManualGPSNode()

    latitude = float(input("Enter latitude: "))
    longitude = float(input("Enter longitude: "))

    manual_gps_node.set_gps(latitude, longitude)

    rclpy.spin(manual_gps_node)
    manual_gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
