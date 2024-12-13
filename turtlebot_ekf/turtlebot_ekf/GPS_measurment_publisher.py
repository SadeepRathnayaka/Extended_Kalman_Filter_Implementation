import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Float32MultiArray
from pyproj import Proj, Transformer


class GPSReadingsPublisher(Node):
    def __init__(self):
        super().__init__("GPS_Readings_Publisher")

        self.sub_ = self.create_subscription(NavSatFix, "/gps/data", self.callback, 10)
        self.pub_ = self.create_publisher(Float32MultiArray, "/gps/measurments", 10)

        self.is_origin   = False
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32644", always_xy=True)


    def callback(self, msg):

        sri_lanka_lat = 6.927079
        sri_lanka_lon = 79.861244

        msg.latitude += sri_lanka_lat
        msg.longitude += sri_lanka_lon

        if not self.is_origin :
            self.origin_x , self.origin_y = self.transformer.transform(msg.longitude, msg.latitude)
            self.is_origin = True
        
        current_x, current_y = self.transformer.transform(msg.longitude, msg.latitude)
        current_x = self.origin_x - current_x 
        current_y = self.origin_y - current_y

        current_coordinates = Float32MultiArray()
        current_coordinates.data.append(current_x)
        current_coordinates.data.append(current_y)
        self.pub_.publish(current_coordinates)


def main(args=None):
    rclpy.init(args=args)

    gps_readings_publisher = GPSReadingsPublisher()

    try:
        rclpy.spin(gps_readings_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_readings_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()