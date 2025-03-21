import sys
import os
sys.path.append('/home/renth/maping_ws/src/my_planning_package')

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from t4ac_msgs.msg import Path

from modules.markers_module import get_point, get_waypoints


class RouteVisualizator(Node):
    """
    Class for visualizing the planner elements (route, init and goal) as
    ROS2 markers in RVIZ
    """

    def __init__(self):
        super().__init__('route_visualizator_node')

        self.create_subscription(
            Odometry,
            "/t4ac/localization/pose",
            self.odometry_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            "/t4ac/planning/goal",
            self.goal_callback,
            10
        )
        self.create_subscription(
            Path,
            "/t4ac/planning/route",
            self.route_callback,
            10
        )

        self.odometry_pub = self.create_publisher(
            Marker, 
            "/t4ac/planning/visualization/odometry", 
            10)
        self.goal_pub = self.create_publisher(
            Marker, 
            "/t4ac/planning/visualization/goal", 
            10)
        self.route_pub = self.create_publisher(
            Marker, 
            "/t4ac/planning/visualization/route", 
            10)

    def odometry_callback(self, odometry):
        odometry_marker = get_point(
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z,
            [1, 0, 1], 0.5, 1
        )
        self.odometry_pub.publish(odometry_marker)

    def goal_callback(self, goal):
        goal_marker = get_point(
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
            [1, 0, 0], -1, 1
        )
        self.goal_pub.publish(goal_marker)

    def route_callback(self, route):
        route_marker = get_waypoints(route.waypoints, [0, 1, 1], -1, 1)
        self.route_pub.publish(route_marker)


def main(args=None):
    rclpy.init(args=args)
    route_visualizator = RouteVisualizator()
    try:
        rclpy.spin(route_visualizator)
    except KeyboardInterrupt:
        pass
    finally:
        route_visualizator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()