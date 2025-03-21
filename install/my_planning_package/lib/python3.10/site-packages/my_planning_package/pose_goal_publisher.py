import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_input_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/t4ac/localization/pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/t4ac/planning/goal', 10)

        timer_period = 1.0  # 发布间隔
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def publish_messages(self):
        # Odometry 模拟当前位置
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = -48.812
        odom_msg.pose.pose.position.y = -140.508
        odom_msg.pose.pose.position.z = 0.0
        self.odom_pub.publish(odom_msg)
        self.get_logger().info('Published current position.')

        # PoseStamped 模拟目标点
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 36.1846
        goal_msg.pose.position.y = -129.643
        goal_msg.pose.position.z = 0.0
        self.goal_pub.publish(goal_msg)
        self.get_logger().info('Published goal.')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
