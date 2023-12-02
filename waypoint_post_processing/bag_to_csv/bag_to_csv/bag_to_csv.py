import sys
import rclpy
from rclpy.node import Node
import csv
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix


class MinimalSubscriber(Node):

    def __init__(self, name):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.file_path = './' + name
        self.fieldnames = ['lat', 'lon']
        # self.create_timer(1.0, self.check_connection)
        self.data = []
        self.csvfile = open(self.file_path, 'w', newline='')
        self.csv_writer = csv.DictWriter(
            self.csvfile, fieldnames=self.fieldnames)
        self.csv_writer.writeheader()

    def listener_callback(self, msg: NavSatFix):
        self.get_logger().info('I heard lat: "%s"' % msg.latitude)
        self.get_logger().info('I heard lon: "%s"' % msg.longitude)
        self.data.append([msg.latitude, msg.longitude])
        lat, lon = [msg.latitude, msg.longitude]
        self.csv_writer.writerow({'lat': lat, 'lon': lon})

    def check_connection(self):
        pub = self.subscription.get_info_by_topic('/gps/fix')
        with open(self.file_path, 'w', newline='') as csvfile:
            csv_writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
            csv_writer.writeheader()
            for lat, lon in data:
                csv_writer.writerow({'lat': lat, 'lon': lon})


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber(sys.argv[1])

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
