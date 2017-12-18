from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import tensorflow as tf
import numpy as np
import rospy
import time

import cv2
from cv_bridge import CvBridge

class TLClassifier(object):
    def __init__(self):
        model_path = rospy.get_param('~model_path', '.')
        rospy.loginfo( '[tl_classifier] model_path: %s', model_path )
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=self.detection_graph)
        # with self.sess as sess:

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image, test = False):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        time_start = time.time()
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={ self.image_tensor: image_np_expanded } )

        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        elapse = time.time() - time_start
        rospy.loginfo( '[tl_classifier] Detection time: %.0fms', elapse * 1000 )
        
        # rospy.loginfo( '[tl_classifier] classes: %s', classes )
        # rospy.loginfo( '[tl_classifier] scores: %s', scores )

        # tx = ['-', 'GREEN', 'RED', 'YELLOW', 'UNKNOWN']
        # rospy.loginfo( '[tl_classifier] light: %s, score: %.2f', tx[ classes[0] ], scores[0] )

        if test:
            return boxes, scores, classes, num

        return self.__get_light_state( scores, classes, num )

    def __get_light_state( self, scores, classes, num ):
        if num > 0:
            claz = []
            for i in range( num ):
                if scores[ i ] >= 0.3:
                    claz.append( classes[ i ] )

            if len( claz ) > 0:
                c = max( claz, key = claz.count )
            else:
                c = 4
        else:
            c = 4

        tx = ['-', 'GREEN', 'RED', 'YELLOW', 'UNKNOWN']
        rospy.loginfo( '[tl_classifier] class: %s', tx[ c ] )

        if c == 1:
            return TrafficLight.GREEN
        if c == 2:
            return TrafficLight.RED
        if c == 3:
            return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
