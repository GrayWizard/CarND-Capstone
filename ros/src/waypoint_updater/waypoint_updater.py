#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Header

import math
import sys
import tf
import copy

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
SAFE_DISTANCE = 32.0

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
        self.traffic_waypoint = -1
        self.current_velocity = 0

        self.decel_limit = rospy.get_param('~decel_limit', -5)

        rospy.spin()

    def update_and_publish(self):
        if self.current_pose and self.base_waypoints:
            next_index = self.closest_forward_waypoint_index()

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            max_index = min(next_index + LOOKAHEAD_WPS, len(self.base_waypoints))
            lane.waypoints = self.base_waypoints[next_index: max_index]

            min_dist_stop = self.current_velocity ** 2 / (2 * (-self.decel_limit) ) + SAFE_DISTANCE
            distance_to_tl = None
            if self.traffic_waypoint != -1 and self.traffic_waypoint > next_index:
                distance_to_tl = self.distance(self.base_waypoints, next_index, self.traffic_waypoint)
                if distance_to_tl < min_dist_stop:
                    lane.waypoints = copy.deepcopy(lane.waypoints[:self.traffic_waypoint - next_index])
                    last_waypoint = lane.waypoints[-1]
                    last_waypoint.twist.twist.linear.x =  0.0
                    for index, point in enumerate(lane.waypoints):
                        dist = self.dist(point.pose.pose.position, last_waypoint.pose.pose.position)
                        dist = max(0.0, dist - SAFE_DISTANCE)
                        velocity = math.sqrt(2 * (-self.decel_limit) * dist)
                        if velocity < 1.0:
                            velocity = 0.0
                        point.twist.twist.linear.x = min(point.twist.twist.linear.x, velocity)

            rospy.logerr('distance_to_tl:{}, min_dist_stop:{},current_velocity:{}'.format(distance_to_tl, min_dist_stop, self.current_velocity))
            self.final_waypoints_pub.publish(lane)

    def closest_forward_waypoint_index(self):
        closest_waypoint_index = self.calculate_closest_waypoint_index()
        if self.has_passed_waypont(closest_waypoint_index):
            return closest_waypoint_index + 1
        else:
            return closest_waypoint_index

    def has_passed_waypont(self, index):
        p1 = self.current_pose.position
        p2 = self.base_waypoints[index].pose.pose.position
        p3 = self.base_waypoints[index + 1].pose.pose.position
        vx, vy = p3.x - p1.x, p3.y - p1.y
        dx, dy = p2.x - p1.x, p2.y - p1.y
        dot = vx * dx + vy * dy
        return (dot < 0)

    def calculate_closest_waypoint_index(self):
        closest_index = 0
        closest_distance = 100000

        for index, waypoint in enumerate(self.base_waypoints):
            position = waypoint.pose.pose.position
            distance = self.dist(self.current_pose.position, position)
            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        return closest_index

    def dist(self, p1,p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def pose_cb(self, msg):
       self.current_pose = msg.pose
       self.update_and_publish()

    def waypoints_cb(self, waypoints):
       self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        pass

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
