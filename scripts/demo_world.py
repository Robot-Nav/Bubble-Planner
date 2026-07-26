#!/usr/bin/env python3
"""Minimal ideal-tracking ROS1 demo for Bubble Planner reproduction."""
import math
import threading

import rospy
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


class DemoWorld:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.lock = threading.Lock()
        self.position = [0.0, 0.0, 1.5]
        self.velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]

        self.cloud_pub = rospy.Publisher(
            "/map_generator/global_cloud", PointCloud2, queue_size=1, latch=True
        )
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        rospy.Subscriber("/bubble_planner/reference/pose", PoseStamped,
                         self.pose_callback, queue_size=1)
        rospy.Subscriber("/bubble_planner/reference/velocity", TwistStamped,
                         self.velocity_callback, queue_size=1)
        rospy.Subscriber("/bubble_planner/reference/acceleration", AccelStamped,
                         self.acceleration_callback, queue_size=1)
        self.points = self.make_environment()
        rospy.Timer(rospy.Duration(0.5), self.publish_cloud)
        rospy.Timer(rospy.Duration(0.01), self.publish_odom)

    @staticmethod
    def make_environment():
        points = []
        resolution = 0.12

        # Ground is represented sparsely; sphere max_radius prevents unbounded growth.
        x = -2.0
        while x <= 18.0:
            y = -7.0
            while y <= 7.0:
                points.append((x, y, 0.0))
                y += 0.35
            x += 0.35

        # Cylindrical pillars create several alternative 3-D passages.
        pillars = [
            (3.5, 0.0, 0.65),
            (6.5, 1.8, 0.70),
            (6.5, -1.8, 0.70),
            (10.0, 0.0, 0.80),
            (13.0, 2.0, 0.65),
            (13.0, -2.0, 0.65),
        ]
        for cx, cy, radius in pillars:
            z = 0.0
            while z <= 4.0:
                angle = 0.0
                while angle < 2.0 * math.pi:
                    points.append((cx + radius * math.cos(angle),
                                   cy + radius * math.sin(angle), z))
                    angle += max(0.08, resolution / radius)
                z += resolution

        # Side fences keep the example bounded but leave vertical room.
        for wall_y in (-5.5, 5.5):
            x = -2.0
            while x <= 18.0:
                z = 0.0
                while z <= 3.2:
                    points.append((x, wall_y, z))
                    z += 0.18
                x += 0.18
        return points

    def pose_callback(self, message):
        with self.lock:
            self.position = [message.pose.position.x,
                             message.pose.position.y,
                             message.pose.position.z]
            self.orientation = [message.pose.orientation.x,
                                message.pose.orientation.y,
                                message.pose.orientation.z,
                                message.pose.orientation.w]

    def velocity_callback(self, message):
        with self.lock:
            self.velocity = [message.twist.linear.x,
                             message.twist.linear.y,
                             message.twist.linear.z]

    def acceleration_callback(self, message):
        with self.lock:
            self.acceleration = [message.accel.linear.x,
                                 message.accel.linear.y,
                                 message.accel.linear.z]

    def publish_cloud(self, _event):
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        try:
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, self.points))
        except rospy.ROSException:
            # A timer callback can race with roslaunch shutting down publishers.
            pass

    def publish_odom(self, _event):
        with self.lock:
            position = list(self.position)
            velocity = list(self.velocity)
            orientation = list(self.orientation)
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = position[0]
        message.pose.pose.position.y = position[1]
        message.pose.pose.position.z = position[2]
        message.pose.pose.orientation.x = orientation[0]
        message.pose.pose.orientation.y = orientation[1]
        message.pose.pose.orientation.z = orientation[2]
        message.pose.pose.orientation.w = orientation[3]
        message.twist.twist.linear.x = velocity[0]
        message.twist.twist.linear.y = velocity[1]
        message.twist.twist.linear.z = velocity[2]
        try:
            self.odom_pub.publish(message)
        except rospy.ROSException:
            # A timer callback can race with roslaunch shutting down publishers.
            pass


if __name__ == "__main__":
    rospy.init_node("bubble_demo_world")
    DemoWorld()
    rospy.loginfo("Demo world ready. In RViz, set a 2D Nav Goal near (16, 0, 1.5).")
    rospy.spin()
