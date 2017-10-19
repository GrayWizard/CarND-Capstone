#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Header

import math
import sys
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.base_waypoints = None
        self.traffic_waypoint = None
        self.obstacle_waypoint = None
        self.current_velocity = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        if self.current_pose and self.base_waypoints:
            closest_index = self.calculate_closest_waypoint_index()
            next_index = self.next_index(closest_index)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = self.base_waypoints[next_index: next_index + LOOKAHEAD_WPS]

            self.final_waypoints_pub.publish(lane)

    def next_index(self, index):
        next_index = index
        p1 = self.current_pose.position
        p2 = self.base_waypoints[index].pose.pose.position
        orientation = self.current_pose.orientation
        heading = math.atan2((p2.y - p1.y), (p2.x - p1.x))
        euler = tf.transformations.euler_from_quaternion((orientation.x,orientation.y,orientation.z,orientation.w))
        yaw = euler[2]
        angle = abs(yaw - heading)

        if angle > math.pi / 4:
            next_index += 1

        return next_index

    def calculate_closest_waypoint_index(self):
        closest_index = 0
        closest_distance = sys.maxint

        for index, waypoint in enumerate(self.base_waypoints):
            position = waypoint.pose.pose.position
            distance = self.dist(self.current_pose.position, position)
            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        return closest_index

    def dist(self, p1,p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    def pose_cb(self, msg):
       self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
       self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
