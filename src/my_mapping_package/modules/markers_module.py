"""
Module that implement functions to generate  util marker objects to represent
different elements of the map in RVIZ
"""
import math 

import rclpy
from rclpy.duration import Duration
import visualization_msgs.msg
import geometry_msgs.msg

from . import monitor_classes

def get_topology(node, waypoint_list, rgb = [0,0,0]):
    """
    Get markers to represent map topology defined by every lane.

    Args:
        waypoint_list: List of waypoints centered in every lane of the map
            at a given distance.

    Returns:
        points: Point markers defining every lane of the map.
    """
    points = visualization_msgs.msg.Marker()
    points.header.frame_id = "map"
    points.header.stamp = node.get_clock().now().to_msg()
    points.ns = "map_visualizator_lanes"
    points.action = visualization_msgs.msg.Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = visualization_msgs.msg.Marker.POINTS
    points.color.r = rgb[0]
    points.color.g = rgb[1]
    points.color.b = rgb[2]
    points.color.a = 1.0
    points.scale.x = 0.2
    points.scale.y = 0.2

    points.lifetime = Duration(seconds=0).to_msg()

    # Fill points.points list with every wp defining lanes
    for waypoint in waypoint_list:
        node_right = monitor_classes.Node3D()
        node_left = monitor_classes.Node3D()
        yaw = waypoint.transform.rotation.yaw
        alpha = yaw
        alpha_radians = math.radians(alpha)
        distance = waypoint.lane_width/2

        k = -1
        if waypoint.lane_id < 0: k = 1

        node_right.x = waypoint.transform.location.x + math.cos(alpha_radians)*distance*(-k)
        node_right.y = waypoint.transform.location.y - math.sin(alpha_radians)*distance*k
        node_right.z = waypoint.transform.location.z

        node_left.x = waypoint.transform.location.x - math.cos(alpha_radians)*distance*(-k)
        node_left.y = waypoint.transform.location.y + math.sin(alpha_radians)*distance*k
        node_left.z = waypoint.transform.location.z

        p1 = geometry_msgs.msg.Point()
        p1.x = node_right.x
        p1.y = node_right.y
        p1.z = node_right.z

        p2 = geometry_msgs.msg.Point()
        p2.x = node_left.x
        p2.y = node_left.y
        p2.z = node_left.z

        points.points.append(p1)
        points.points.append(p2)
    return points

def get_lane(node, lane, rgb = [0,0,0], name = "", marker_type = 4,
             scale = 0.2, extra_z = 0, lifetime = 0.2, id=0, a=1):
    """
    Get lane marker to represent monitorized lane.

    Args:
        lane: Monitorized lane to represent of type t4ac_msgs.msg.Lane()
        rbg: Colour to represent the lane
        name: Differential name of the marker for namespace
        marker_type: (int) Type of marker, usually LINE_STRIP (4) or POINTS (8)
        scale: Scale for the marker
        extra_z: (int) Extra value for coordinate z
        lifetime: Lifetime of the marker in seconds
        id: (int)
        a: (float) 0 < a < 1

    Returns:
        lane_marker_right: Marker to represent right way of monitorized lane.
        lane_marker_left: Marker to represent left way of monitorized lane.
    """
    # Right way of lane 
    lane_marker_right = visualization_msgs.msg.Marker()
    lane_marker_right.header.frame_id = "map"
    lane_marker_right.header.stamp = node.get_clock().now().to_msg()
    # lane_marker_right.ns = str(lane.role)+"lane_marker_right"
    lane_marker_right.ns = "lane_" + name + "_marker_right"
    lane_marker_right.action = visualization_msgs.msg.Marker.ADD
    lane_marker_right.pose.orientation.w = 1.0
    lane_marker_right.id = id
    lane_marker_right.type = marker_type
    lane_marker_right.color.r = rgb[0]
    lane_marker_right.color.g = rgb[1]
    lane_marker_right.color.b = rgb[2]
    lane_marker_right.color.a = a
    lane_marker_right.scale.x = scale
    lane_marker_right.scale.y = scale
    lane_marker_right.lifetime = Duration(seconds=lifetime).to_msg() 

    for node in lane.right.way:
        point = geometry_msgs.msg.Point()
        point.x = node.x
        point.y = node.y
        point.z = node.z + extra_z
        lane_marker_right.points.append(point)

    # Left way of lane
    lane_marker_left = visualization_msgs.msg.Marker()
    lane_marker_left.header.frame_id = "map"
    lane_marker_left.header.stamp = node.get_clock().now().to_msg()
    # lane_marker_left.ns = str(lane.role)+"lane_marker_left"
    lane_marker_left.ns = "lane_" + name + "_marker_left"
    lane_marker_left.action = visualization_msgs.msg.Marker.ADD
    lane_marker_left.pose.orientation.w = 1.0
    lane_marker_left.id = id
    lane_marker_left.type = marker_type
    lane_marker_left.color.r = rgb[0]
    lane_marker_left.color.g = rgb[1]
    lane_marker_left.color.b = rgb[2]
    lane_marker_left.color.a = a
    lane_marker_left.scale.x = scale
    lane_marker_left.scale.y = scale
    lane_marker_left.lifetime = Duration(seconds=lifetime).to_msg() 

    for node in lane.left.way:
        point = geometry_msgs.msg.Point()
        point.x = node.x
        point.y = node.y
        point.z = node.z + extra_z
        lane_marker_left.points.append(point)

    return lane_marker_right, lane_marker_left

def get_nodes(node, nodes, rgb = [0,0,0], name = "", marker_type = 4,
             scale = 0.2, extra_z = 0, lifetime = 0.2):
    """
    Get nodes marker to represent a list of nodes in RVIZ

    Args:
        nodes: List of monitor_classes.Node3D
        rbg: Colour to represent the lane
        name: Differential name of the marker for namespace
        marker_type: (int) Type of marker, usually LINE_STRIP (4) or POINTS (8)
        scale: Scale for the marker
        extra_z: (int) Extra value for coordinate z
        lifetime: Lifetime of the marker in seconds

    Returns:
        nodes: Marker to represent a way in RVIZ
    """
    nodes_marker = visualization_msgs.msg.Marker()
    nodes_marker.header.frame_id = "map"
    nodes_marker.header.stamp = node.get_clock().now().to_msg()
    nodes_marker.ns = "nodes_marker" + name
    nodes_marker.action = visualization_msgs.msg.Marker.ADD
    nodes_marker.pose.orientation.w = 1.0
    nodes_marker.id = 0
    nodes_marker.type = marker_type
    nodes_marker.color.r = rgb[0]
    nodes_marker.color.g = rgb[1]
    nodes_marker.color.b = rgb[2]
    nodes_marker.color.a = 1.0
    nodes_marker.scale.x = scale
    nodes_marker.lifetime = Duration(seconds=lifetime).to_msg() 

    for node in nodes:
        point = geometry_msgs.msg.Point()
        point.x = node.x
        point.y = -node.y
        point.z = node.z + extra_z
        nodes_marker.points.append(point)
    return nodes_marker
