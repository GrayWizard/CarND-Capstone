#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from collections import namedtuple

from multiprocessing import Pool


STATE_COUNT_THRESHOLD = 3
MAX_DISTANCE = 80

Point = namedtuple('Point', ['x', 'y'])

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	
	self.n_predictions = 0
	self.n_correct = 0

	self.light_classifier = TLClassifier()
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

	rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
	self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def dist(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_index = 0
        closest_distance = 100000

        for index, waypoint in enumerate(self.waypoints.waypoints):
            position = waypoint.pose.pose.position
            distance = self.dist(pose, position)
            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        return closest_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def has_passed_point(self, p1, p2, p3):
        vx, vy = p3.x - p1.x, p3.y - p1.y
        dx, dy = p2.x - p1.x, p2.y - p1.y
        dot = vx * dx + vy * dy
        return dot < 0

    def find_closest_visible_traffic_light(self, pose, stop_line_positions):
        closest_index = -1
        closest_distance = 100000
        for index, stop_line in enumerate(stop_line_positions):
            distance = self.dist(pose.position, Point(*stop_line))
            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        next_index = (closest_index + 1) % len(stop_line_positions)
        passed = self.has_passed_point(pose.position, Point(*stop_line_positions[closest_index]),
                                       Point(*stop_line_positions[next_index]))

        if passed:
            return next_index

        return closest_index

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not (self.pose and self.waypoints):
            return -1, TrafficLight.UNKNOWN

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        car_position = self.get_closest_waypoint(self.pose.pose.position)

        #TODO find the closest visible traffic light (if one exists)

        light_wp = -1
        for i, stop_line in enumerate(stop_line_positions):
            distance = self.dist(self.pose.pose.position, Point(*stop_line))
            if distance > MAX_DISTANCE:
                continue
            stop_line_wp = self.get_closest_waypoint(Point(*stop_line))
            if stop_line_wp > car_position:
                if light_wp == -1 or light_wp > stop_line_wp:
                    light_wp = stop_line_wp
                    light = self.lights[i]

        if light:
            state = light.state # just for testing and verification

	    state = self.get_light_state(light) # comment this line to ignore classifier

	    self.n_predictions += 1
	    
	    if light.state is not None:
		# RED / NOT RED
		if  light.state == state or (light.state > 0 and state > 0):
		    self.n_correct += 1

	    #rospy.logerr('Prediction accuracy: {:.1f}%'.format(100 * self.n_correct/self.n_predictions))
	    #rospy.logerr('Predicted: {} - Ground truth: {}'.format(state, light.state))

            return light_wp, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
