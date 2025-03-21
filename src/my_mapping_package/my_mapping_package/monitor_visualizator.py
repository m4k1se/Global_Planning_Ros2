#!/usr/bin/env python3
"""
Module to implement callbacks for monitorized elements. Every time a
monitorized element is published, there is a callback to represent it
in RVIZ
"""
import sys
import os
sys.path.append('/home/renth/maping_ws/src/my_mapping_package')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from t4ac_msgs.msg import MonitorizedLanes, MonitorizedRegElem
from modules import monitor_classes
from modules import markers_module


class MonitorVisualizator(Node):
    """
    Class for the monitor visualizator.
    Represents all the monitorized elements in ROS topics as markers, to 
    be visualized in RVIZ.
    """

    def __init__(self):
        super().__init__("monitor_visualizator_node")

        # QoS Profile (类似 queue_size，但更稳定)
        qos_profile = QoSProfile(depth=10)

        # ROS2 Subscribers
        self.lanes_monitor_sub = self.create_subscription(
            MonitorizedLanes, "/t4ac/mapping/monitor/lanes",
            self.lanes_callback, qos_profile)
        self.intersections_monitor_sub = self.create_subscription(
            MonitorizedLanes, "/t4ac/mapping/monitor/intersections",
            self.intersections_callback, qos_profile)
        self.regElems_monitor_sub = self.create_subscription(
            MonitorizedRegElem, "/t4ac/mapping/monitor/regElems",
            self.regElems_callback, qos_profile)

        # ROS2 Publishers
        self.lanes_monitor_visualizator_pub = self.create_publisher(
            MarkerArray, "/t4ac/mapping/monitor/lanes_marker", qos_profile)
        self.intersections_monitor_visualizator_pub = self.create_publisher(
            MarkerArray, "/t4ac/mapping/monitor/intersections_marker", qos_profile)
        self.regElems_monitor_visualizator_pub = self.create_publisher(
            Marker, "/t4ac/mapping/monitor/regElems_marker", qos_profile)

    def lanes_callback(self, lanes):
        """
        Callback function called when lanes are published by the map_monitor
        in /t4ac/mapping/monitor/lanes
        """
        standard_lane_markers = MarkerArray()
        for lane in lanes.lanes:
            if lane.role == "current_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                    self, lane, [1, 0, 0], "current_front_", 4, 0.2, 0.1, 0.5, 0)

            elif lane.role == "current_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                    self, lane, [0, 0, 1], "current_back_", 4, 0.2, 0.1, 0.5, 2)

            elif lane.role == "right_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                    self, lane, [1, 1, 0], "right_front_", 8, 0.4, 0.1, 0.5, 4)

            elif lane.role == "right_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                    self, lane, [1, 1, 0], "right_back_", 8, 0.4, 0.1, 0.5, 6, 0.5)

            elif lane.role == "left_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                    self, lane, [1, 1, 0], "left_front_", 8, 0.4, 0.1, 0.5, 8)

            elif lane.role == "left_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                    self, lane, [1, 1, 0], "left_back_", 8, 0.4, 0.1, 0.5, 10, 0.5)

        self.lanes_monitor_visualizator_pub.publish(standard_lane_markers)

    def intersections_callback(self, intersection_lanes):
        """
        Callback function called when intersection_lanes are published by 
        the map_monitor in /t4ac/mapping/monitor/intersections
        """
        intersection_lane_markers = MarkerArray()
        i = 0
        for lane in intersection_lanes.lanes:
            if lane.role == "merge":
                intersection_lane_markers.markers += markers_module.get_lane(
                    lane, [1, 1, 0], f"merge_{i}", 4, 0.2, 0.05, 1, i)
                i += 1

            elif lane.role == "split":
                intersection_lane_markers.markers += markers_module.get_lane(
                    lane, [1, 0.5, 0], f"split_{i}", 4, 0.2, 0.05, 1, i)
                i += 1

            elif lane.role == "cross":
                intersection_lane_markers.markers += markers_module.get_lane(
                    lane, [1, 0, 1], f"cross_{i}", 4, 0.2, 0.05, 1, i)
                i += 1

        self.intersections_monitor_visualizator_pub.publish(intersection_lane_markers)

    def regElems_callback(self, regElems):
        """
        Callback function called when regElems are published by 
        the map_monitor in /t4ac/mapping/monitor/regElems
        """
        nodes = []
        if len(regElems.reg_elems) > 0:
            for regElem in regElems.reg_elems:
                node = monitor_classes.Node3D()
                node.x = regElem.location.x
                node.y = regElem.location.y
                node.z = regElem.location.z
                nodes.append(node)
                for landmark in regElem.landmarks:
                    node = monitor_classes.Node3D()
                    node.x = landmark.location.x
                    node.y = landmark.location.y
                    node.z = landmark.location.z
                    nodes.append(node)

            landmarks_marker = markers_module.get_nodes(
                self, nodes, [1, 0, 0], "1", 8, 0.5, 1, 0.2)
            self.regElems_monitor_visualizator_pub.publish(landmarks_marker)


def main():
    rclpy.init()
    node = MonitorVisualizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
