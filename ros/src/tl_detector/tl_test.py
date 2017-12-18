#!/usr/bin/env python
import rospy
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import cv2
import math
import os

import numpy as np
# from PIL import Image
# from PIL import ImageDraw
# from PIL import ImageColor

# ***************************************************************************
# ******************** TEST *************************************************
# ***************************************************************************
class TLTest(object):
    def __init__(self):
        rospy.init_node('tl_test')
        
        self.light_classifier = TLClassifier()

        rospy.Subscriber('/image_raw', Image, self.image_cb)

        self.detection_pubs = rospy.Publisher('/detection', Image, queue_size=1)

        self.bridge = CvBridge()

        self.collect_count = 0
        self.collect_path = rospy.get_param('~collect_path')

        rospy.spin()

    def image_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2( msg, "rgb8" )

        #Get classification
        boxes, scores, classes, num = \
            self.light_classifier.get_classification( cv_image, test = True )

        self.print_info( scores, classes, num )

        self.pub_detection( cv_image, boxes, scores, classes )

    def print_info( self, scores, classes, num ):
        if num > 0:
            claz = []
            for i in range( num ):
                if scores[ i ] >= 0.3:
                    claz.append( classes[ i ] )

            if len( claz ) > 0:
                c = max( claz, key = claz.count )
            else:
                c = 4
            c1 = classes[ 0 ]
            score1 = scores[ 0 ]
        else:
            c = 4
            c1 = 4
            score1 = 0

        tx = ['-', 'GREEN', 'RED', 'YELLOW', 'UNKNOWN']
        rospy.loginfo( '[tl_test] class: %s, [%s: %.2f]', tx[ c ], tx[ c1 ], score1 )

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
        
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        
        return box_coords

    def draw_boxes(self, image, boxes, classes, thickness=4):
        """Draw bounding boxes on the image"""
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            class_id = int(classes[i])
            if class_id == 1: # Green
                color = (0, 255, 0)
                text = 'Green'
            elif class_id == 2: # Red
                color = (255, 0, 0)
                text = 'Red'
            elif class_id == 3: # Yellow
                color = (255, 255, 0)
                text = 'Yellow'
            else:
                color = (255, 255, 255)
                text = 'Unknown'
            cv2.rectangle(image, (left, top), (right, bot), color, thickness)

            # Draw Text
            font                   = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = ( left, bot )
            fontScale              = 1
            fontColor              = color
            lineType               = 2

            cv2.putText( image, text, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)


    def pub_detection( self, image, boxes, scores, classes ):
        confidence_cutoff = 0.3
        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)

        # The current box coordinates are normalized to a range between 0 and 1.
        # This converts the coordinates actual location on the image.
        height, width, c = image.shape
        box_coords = self.to_image_coords(boxes, height, width)

        # Each class with be represented by a differently colored box
        self.draw_boxes(image, box_coords, classes)

        # self.save_image( image )

        self.detection_pubs.publish( self.bridge.cv2_to_imgmsg( image, "rgb8" ) )

    def save_image( self, image ):
        file = os.path.join( self.collect_path, 'site_{:0>4d}.png'.format( self.collect_count ) )
        self.collect_count += 1
        cv2.imwrite( file, cv2.cvtColor( image, cv2.COLOR_RGB2BGR ) )

if __name__ == '__main__':
    TLTest()