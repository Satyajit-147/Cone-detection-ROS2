import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')

        # Publisher: sends velocity commands to robot
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: listens to LaserScan for obstacle detection
        self.scan_sub = self.create_subscription(LaserScan, '/laser_scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        self.get_logger().info(f"Closest object: {min_distance:.2f} m")

        cmd_vel = Twist()
        if min_distance < 0.5:
            self.get_logger().warn("Obstacle too close! Stopping robot.")
            cmd_vel.linear.x = 0.0
        else:
            cmd_vel.linear.x = 0.2  # move forward

        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

