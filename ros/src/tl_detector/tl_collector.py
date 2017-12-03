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
import os


class TLCollector(object):
    def __init__(self):
        rospy.init_node('tl_collector')

        rospy.Subscriber('/image_color', Image, self.image_cb)

        self.bridge = CvBridge()
        
        self.collect_count = 1
        self.collect_path = rospy.get_param('~collect_path')

        self.traffic_wp = -1
        self.camera_img = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            if self.camera_img is not None:
                cv_image = self.bridge.imgmsg_to_cv2( self.camera_img, "bgr8" )
                file = os.path.join( self.collect_path, 'sim_{:0>4d}.png'.format( self.collect_count ) )
                self.collect_count += 1
                cv2.imwrite( file, cv_image )

            rate.sleep()

    def image_cb(self, msg):
        self.camera_img = msg


if __name__ == '__main__':
    try:
        TLCollector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
