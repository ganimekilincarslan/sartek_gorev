import json
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GcsWaypointSender(Node):
    def __init__(self):
        super().__init__('gcs_waypoint_sender')
        self.pub = self.create_publisher(String, '/yer_istasyonu/waypoints', 10)

        # Farklı irtifalarda 6 adet Local ENU waypoint (X=Doğu, Y=Kuzey, Z=Yukarı m)
        self.route = [
            {'x': 150.0,  'y': 100.0,  'z': 50.0},
            {'x': 300.0,  'y': 200.0,  'z': 75.0},
            {'x': 250.0,  'y': -100.0, 'z': 90.0},
            {'x': 50.0,   'y': -200.0, 'z': 65.0},
            {'x': -150.0, 'y': -50.0,  'z': 80.0},
            {'x': -50.0,  'y': 150.0,  'z': 55.0},
        ]

    def send_bulk(self):
        msg = String()
        msg.data = json.dumps(self.route)
        self.pub.publish(msg)
        self.get_logger().info(f'TOPLU MOD: {len(self.route)} waypoint tek seferde gönderildi.')

    def send_piece_by_piece(self, delay_sec=3.0):
        self.get_logger().info(f'PARÇA PARÇA MOD: {len(self.route)} waypoint {delay_sec}s arayla gönderiliyor...')
        for i, wp in enumerate(self.route, start=1):
            msg = String()
            msg.data = json.dumps(wp)
            self.pub.publish(msg)
            self.get_logger().info(f'Gönderilen WP {i}: {wp}')
            time.sleep(delay_sec)


def main(args=None):
    rclpy.init(args=args)
    node = GcsWaypointSender()

    # Görev düğümünün topic'e bağlanması için kısa bekleme
    time.sleep(1.0)

    if '--parca' in sys.argv:
        node.send_piece_by_piece(delay_sec=3.0)
    else:
        node.send_bulk()

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()