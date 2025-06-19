import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSPublisher(Node):
    def __init__(self):
        super().__init__("gps_publisher")
        self.publisher_ = self.create_publisher(NavSatFix, "uav_gps", 10)

        # Create a TCP server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("0.0.0.0", 5000))  # Match port with ESP8266
        self.sock.listen(1)
        self.get_logger().info("Waiting for ESP8266 connection...")
        self.conn, _ = self.sock.accept()
        self.get_logger().info("ESP8266 connected!")
        self.conn_file = self.conn.makefile("r")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            data = self.conn_file.readline().strip()
            print(f"Raw data received: '{data}'")
            if not data:
                return

            parts = data.split(",")
            if len(parts) != 5:
                self.get_logger().warn(f"Malformed data: {data}")
                return
            num_sats, lat_str, lon_str, alt_str, hdop = parts
            print("Quality: " + hdop + "\n")

            msg = NavSatFix()
            msg.latitude = float(lat_str)
            msg.longitude = float(lon_str)
            msg.altitude = max(0, float(alt_str) - 90.0)
            msg.header.frame_id = "gps"
            self.publisher_.publish(msg)
            # print("Num of satellites: " + num_sats + "\n")
            # print("Quality: " + hdop + "\n")
            self.get_logger().info(
                f"Published: {msg.latitude}, {msg.longitude}, {msg.altitude}"
            )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
