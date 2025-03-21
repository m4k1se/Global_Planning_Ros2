#!/usr/bin/env python3
"""
Map Visualizator without using PythonAPI

Last mod: Alejandro D. 4/5/22
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker

sys.path.append('/home/renth/maping_ws/src/my_mapping_package')
from map_parser.map_object import MapObject
from modules import markers_module


class MapVisualizator(Node):
    """
    ROS2 节点：用于可视化地图，并在地图发生变化时更新
    """

    def __init__(self):
        super().__init__("map_visualizator_node")

        # 声明参数（如果未提供则默认为 ""）
        self.declare_parameter("t4ac/mapping/map_name", "Town10HD")
        self.declare_parameter("t4ac/mapping/map_path", "/home/renth/opendrive-mapping-planning/mapping_layer/maps/xodr/")

        # 读取初始参数
        self.map_path = self.get_parameter("t4ac/mapping/map_path").value
        self.map_name = self.get_parameter("t4ac/mapping/map_name").value
        self.previous_map_name = None

        # 创建发布者
        self.map_visualizator_pub = self.create_publisher(
            Marker, "/t4ac/mapping/map/lanes_marker", 10)

        # 初始化地图
        self.load_map()

        # 运行主循环
        self.timer = self.create_timer(1.0, self.update_map)  # 1Hz 更新地图

    def load_map(self):
        """ 加载地图并生成标记 """
        self.map_object = MapObject(self.map_name, self.map_path)
        self.waypoints = self.map_object.map_waypoints
        self.lane_markers = markers_module.get_topology(self, self.waypoints, [192.0, 192.0, 192.0])

    def update_map(self):
        """ 监测地图是否发生变化，若变化则更新 """
        self.map_name = self.get_parameter("t4ac/mapping/map_name").value
        if self.map_name != self.previous_map_name:
            self.get_logger().info(f"Loading new map: {self.map_name}")
            self.load_map()
            self.map_visualizator_pub.publish(self.lane_markers)
            self.previous_map_name = self.map_name


def main():
    rclpy.init()
    node = MapVisualizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    print("Start Map Visualizator Node")
    main()
