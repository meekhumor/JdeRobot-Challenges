import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publisher node has started')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello! ROS2 is fun'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()