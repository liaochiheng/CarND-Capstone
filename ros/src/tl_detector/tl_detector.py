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

STATE_COUNT_THRESHOLD = 3
LIGHT_AHEAD_WPS = 200

# The car should stop ahead red light about 27.m
STOP_AHEAD_DIST = 27.

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_wps = None
        self.camera_image = None
        self.lights = []
        self.light_wps = None

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
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_wps = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if not self.pose or not self.base_wps:
            return

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

    def get_closest_waypoint(self, x, y, z):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.base_wps

        """
        #TODO implement
        if self.base_wps == None:
            return None

        dist = float( "inf" )
        wpi = None
        for i, wp in enumerate( self.base_wps.waypoints ):
            p = wp.pose.pose.position
            d = math.sqrt( ( p.x - x ) ** 2 + ( p.y - y ) ** 2 + ( p.z - z ) ** 2 )
            if d < dist:
                wpi = i
                dist = d

        return wpi

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

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
        
        pos = self.pose.pose.position
        car_wp = self.get_closest_waypoint( pos.x, pos.y, pos.z )

        return self.find_upcoming_light_test( car_wp )

        #TODO find the closest visible traffic light (if one exists)
        # light_wp = find_upcoming_light( car_wp )
        # if light_wp != -1:
        #     state = self.get_light_state(light)
        #     return light_wp, state
        # # self.waypoints = None
        # return -1, TrafficLight.UNKNOWN

    def find_upcoming_light_test( self, car_wp ):
        num_wps = len( self.base_wps.waypoints )

        light_wps = []
        for light in self.lights:
            p = light.pose.pose.position
            wp = self.get_closest_waypoint( p.x, p.y, p.z )
            # Find real stop location of light
            wp = self.get_stop_waypoint( wp )

            light_wps.append( wp )

        # find upcoming light
        light_wp = -1
        state = TrafficLight.UNKNOWN
        
        light_dist = num_wps * 2
        for i, wp in enumerate( light_wps ):
            dist = wp - car_wp if wp > car_wp else wp + num_wps - car_wp
            if dist < light_dist:
                light_wp = wp
                state = self.lights[ i ].state
                light_dist = dist

        ss = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']
        rospy.loginfo( '[tl_detector] upcoming: %d - %d = %d, dist = %.2f, state = %s', \
            light_wp, car_wp, light_dist, \
            self.distance( self.base_wps.waypoints, car_wp, light_wp ), ss[ state ] )

        if light_dist > LIGHT_AHEAD_WPS:
            light_wp = -1
            state = TrafficLight.UNKNOWN
        #else:
        # ss = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']
        # rospy.loginfo( '[tl_detector] upcoming: %d - %d = %d, state = %s', \
        #     light_wp, car_wp, light_dist, ss[ state ] )

        return light_wp, state

    # Find the real stop waypoint of 'light_wp', which should locate ahead about 27m
    def get_stop_waypoint( self, light_wp ):
        waypoints = self.base_wps.waypoints
        num_wps = len( waypoints )
        wp1_last, wp1 = light_wp, light_wp
        dist_last, dist = 0, 0

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        
        while dist < STOP_AHEAD_DIST:
            wp0 = wp1 - 1 if wp1 >= 1 else num_wps - 1
            dist_last = dist
            dist += dl( waypoints[ wp0 ].pose.pose.position, waypoints[ wp1 ].pose.pose.position )
            wp1_last = wp1
            wp1 = wp0

        return wp1 if dist - STOP_AHEAD_DIST < STOP_AHEAD_DIST - dist_last else wp1_last

    def _distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, waypoints, wp1, wp2):
        num = len( waypoints )
        wp1, wp2 = wp1 % num, wp2 % num
        if wp2 > wp1:
            return self._distance( waypoints, wp1, wp2 )
        else:
            num_wps = len( self.base_wps.waypoints )
            return self._distance( waypoints, wp1, num_wps - 1 ) + \
                    self._distance( waypoints, 0, wp2 )

    def find_upcoming_light( self, car_wp ):
        # Calc waypoint index for lights from config, calc only once
        if not self.light_wps:
             # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']
            self.light_wps = []
            for p in stop_line_positions:
                self.light_wps.append( self.get_closest_waypoint( p[ 0 ], p[ 1 ], 0. ) )
            self.light_wps.sort()

        # find upcoming light
        light_wp = -1
        light_dist = -1
        for wp in self.light_wps:
            if wp > car_wp:
                light_wp = wp
                light_dist = wp - car_wp
                break
        else:
            light_wp = self.light_wps[ 0 ]
            light_dist = light_wp + len( self.base_wps ) - car_wp

        return light_wp if light_dist <= LIGHT_AHEAD_WPS else -1

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
