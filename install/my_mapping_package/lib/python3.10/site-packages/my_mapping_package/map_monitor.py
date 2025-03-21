#!/usr/bin/env python3
"""
Receive path planning route, ego_vehicle_position and xodr map.
With that info calculate monitorized elements.
"""

import sys
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from t4ac_msgs.msg import MonitorizedLanes, MonitorizedRegElem, Path

sys.path.append('/home/renth/maping_ws/src/my_mapping_package')
print(sys.path)
from map_parser.map_object import MapObject

from modules import route_module
from modules import calculus_module
from modules import monitor_module


class MapMonitor(Node):
    """
    Class for the map monitor. 
    Here are two main callbacks: one when a new route is published, and another
    when localization of the vehicle is published (localization is assumed 
    to be constantly published).
    """

    def __init__(self):
        super().__init__("map_monitor_node")

        # Get parameters
        self.declare_parameter("t4ac/mapping/map_name", "Town10HD")
        self.declare_parameter("t4ac/mapping/map_path", "/home/renth/opendrive-mapping-planning/mapping_layer/maps/xodr/")
        
        map_name = self.get_parameter("t4ac/mapping/map_name").value
        map_path = self.get_parameter("t4ac/mapping/map_path").value

        # Map
        self.map_object = MapObject(map_name, map_path)
        self.route_segment_centers = None
        self.segment_index = -1
        self.ego_vehicle_waypoint = None
        self.waypoint_route = []
        self.flag_goal_reached = 0
        self.n_min = 15
        self.monitor_flag = 0

        # QoS Profile
        qos_profile = rclpy.qos.QoSProfile(depth=10)

        # Monitor ROS Publishers
        self.lanes_monitor_pub = self.create_publisher(
            MonitorizedLanes, "/t4ac/mapping/monitor/lanes", qos_profile)
        self.intersections_monitor_pub = self.create_publisher(
            MonitorizedLanes, "/t4ac/mapping/monitor/intersections", qos_profile)
        self.regElems_monitor_pub = self.create_publisher(
            MonitorizedRegElem, "/t4ac/mapping/monitor/regElems", qos_profile)

        # Monitor ROS Subscribers
        self.route_sub = self.create_subscription(
            Path, "/t4ac/planning/route", self.route_callback, qos_profile)
        self.localization_sub = self.create_subscription(
            Odometry, "/t4ac/localization/pose", self.localization_callback, qos_profile)

    ### Route Callback ###
    def route_callback(self, path_route):
        """
        Callback function called when a route is published by path planner.

        Args:
            path_route: Route of type nav_msgs/Path.msg

        Returns: 
            Set route_location and route_waypoint with new route published.
            Also check if vehicle is inside the route and in which segment.
        """
        self.waypoint_route = route_module.path_to_waypoint_route(
            path_route, self.map_object)
        if self.ego_vehicle_waypoint is not None:
            self.route_segment_centers = (
                route_module.calculate_route_segment_centers(self.waypoint_route))
            self.segment_index = route_module.get_route_segment_index(
                self.route_segment_centers, self.ego_vehicle_waypoint)
            self.monitor_flag = 1
        else:
            self.segment_index = -1

    ### Ego_vehicle_position Callback ###
    def localization_callback(self, ego_vehicle_odometry):
        """
        Callback function called when an ego_vehicle position is published.

        Args:
            ego_vehicle_odometry: Current local UTM position of the 
                vehicle of type nav_msgs/Odometry.msg

        Returns: 
            Activates map monitor if possible.
        """
        self.ego_vehicle_waypoint = self.map_object.get_waypoint(
            ego_vehicle_odometry.pose.pose.position.x,
            ego_vehicle_odometry.pose.pose.position.y,
            ego_vehicle_odometry.pose.pose.position.z)
        print(ego_vehicle_odometry.pose.pose.position.x)
        print(ego_vehicle_odometry.pose.pose.position.y)
        print(ego_vehicle_odometry.pose.pose.position.z)
        # Calculate monitoring waypoints based on velocity
        n_max = calculus_module.braking_n_distance(ego_vehicle_odometry)
        if n_max < self.n_min:
            n_max = self.n_min
        n2_max = int(n_max / 2)

        if len(self.waypoint_route) > 0:
            if self.route_segment_centers is not None:
                self.segment_index = route_module.get_route_segment_index(
                    self.route_segment_centers, self.ego_vehicle_waypoint)

        if self.segment_index >= 0:
            if self.segment_index == (len(self.waypoint_route) - 1):
                if self.flag_goal_reached == 0:
                    self.get_logger().info("Goal reached!")
                    self.flag_goal_reached = 1
            else:
                if self.monitor_flag == 1:
                    self.get_logger().info("Monitoring...")
                    self.monitor_flag = 0

                n1 = n_max if (self.segment_index + n_max) < len(self.waypoint_route) else len(self.waypoint_route) - self.segment_index
                n2 = self.segment_index if (self.segment_index - n2_max) < 0 else n2_max

                # Publish monitored lanes
                lanes = monitor_module.calculate_lanes(
                    self, self.map_object.map_waypoints, self.map_object.map_kdtree, 
                    self.segment_index, self.waypoint_route[:], n1, n2)
                if lanes:
                    self.lanes_monitor_pub.publish(lanes)

                # Publish monitored intersections
                intersection_lanes = monitor_module.calculate_intersections(
                    self, self.waypoint_route[:], self.segment_index, n1, 
                    self.map_object, self.map_object.roads)
                if intersection_lanes:
                    self.intersections_monitor_pub.publish(intersection_lanes)


def main(args=None):
    rclpy.init(args=args)
    node = MapMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
