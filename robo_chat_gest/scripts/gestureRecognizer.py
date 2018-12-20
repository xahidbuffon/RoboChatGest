#! /usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/

Class for detecting left and right hand-gestures using deep object detection model
currently we are using SSD based model that was described in this paper: 
    https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21837

"""

# ros/python/opencv/tensorflow libraries
import os
import sys
import cv2
import yaml
import rospy
import roslib
import argparse
import numpy as np
import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL']='2' #export TF_CPP_MIN_LOG_LEVEL=2


class HandGestRecognition():
    """
      class for driving the deep object detector efficiently in real-time 
      detects {left, right} hand-gestures and maps them into tokens
      returns the tokens and corresponding boxes
    """

    def __init__(self):
        """
          initialize variables and flags
        """ 
        #self.data_dir = '/home/aqua/model_data_/robo_gest_cnn/'
        self.data_dir = '/home/aqua/model_data_/robo_gest_ssd/'
        self.min_score_thresh = 0.4
        self.CNN_init()
        

    def CNN_init(self):
        """
          load saved tensorflow graph and model parameters
          just load once, DONOT load every time for each frame
	    then it'll be too slow
        """ 
        frozen_model_path = self.data_dir+'frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(frozen_model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        ops = self.detection_graph.get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        self.tensor_dict = {}
        for key in ['detection_boxes', 'detection_scores','detection_classes']:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                self.tensor_dict[key] = self.detection_graph.get_tensor_by_name(tensor_name)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.sess = tf.Session(graph=self.detection_graph)





    def Detect_gest(self, image):
        """
          given an image, return the detected objects {Number of objects, boxes, classes, confidence scores} 
        """ 
        output_dict = self.sess.run(self.tensor_dict, feed_dict={self.image_tensor: np.expand_dims(image, 0)})
        output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
        output_dict['detection_scores'] = output_dict['detection_scores'][0]
        boxes, classes, scores = output_dict['detection_boxes'], output_dict['detection_classes'], output_dict['detection_scores']
        N_obj = sum(scores>self.min_score_thresh)
        return N_obj, boxes[:N_obj], classes[:N_obj], scores[:N_obj]
        



    def Get_gest(self, frame, one_hand_only=False):
        """
          given an image
               > First detect the objects {Number of objects, boxes, classes, scores}
               > Then filter the detections to remove noise/missed detections 
               > returns left_token, left_box, right_token, right_box, success_flag
        """ 
        # *** remember to convert the frame to RGB from BGR
        im_height, im_width, _ = frame.shape
        image = cv2.cvtColor(cv2.resize(frame, (400, 400)), cv2.COLOR_BGR2RGB)

        # Detect the objects {Number of objects, boxes, classes, scores}
        Nobs, boxes, classes, scores = self.Detect_gest(image)

        # Detect the objects {Number of objects, boxes, classes, scores}
        if Nobs<=0:
            return None, None, None, None, False
        else:
            """
            Filter the detections to remove noise/missed detections
            This is actually a critical part, the Barbados 2018 ocean-tests **cked up because this wasn't designed right
              "it is easy to blame the detector or retrain it a million times, 
                                       but before that, make sure the practicalities are handled" -- Frustrated Jahid
            """
            sc_1, sc_2 = 0, 0
            cl_1, sc_1, id_1 = classes[0], scores[0], 0
            ymin, xmin, ymax, xmax = tuple(boxes[id_1].tolist())
            left_box = (int(xmin*im_width), int(xmax*im_width), int(ymin*im_height), int(ymax*im_height))

            if one_hand_only:
                left_token = cl_1%10
                if (sc_1 > self.min_score_thresh):
                    # copy the same thing to other hand, if we are using just one
                    return left_token, left_box, left_token, left_box, True
                else:
                    return None, None, None, None, False
    
            # we can have same gesture on both hands, but both hands should be horizontally far enough
            for i in xrange(1, scores.shape[0]): 
                if classes[i] != cl_1:
                    # we are good if we have different classes
                    cl_2, sc_2, id_2 = classes[i], scores[i], i
                    break
                else:
                    # otherwise the hands have to be atleast 50 pixels apart
                    # this helps to ignore multiple detections from the same hand
                    ymin2, xmin2, ymax2, xmax2 = tuple(boxes[i].tolist())
                    if (abs(xmin+xmax-xmin2-xmax2)*im_width > 50):
                        cl_2, sc_2, id_2 = classes[i], scores[i], i
                        break

            if (sc_1 > self.min_score_thresh and sc_2 > self.min_score_thresh):
                # if both detections are reasonably confident, assign {left, right} tokens and return 
                ymin, xmin, ymax, xmax = tuple(boxes[id_2].tolist())
                right_box = (int(xmin*im_width), int(xmax*im_width), int(ymin*im_height), int(ymax*im_height))
                # 10 is actually 0, 1-9 remains same. so perform MOD operation    
                left_token, right_token = cl_1%10, cl_2%10
                # assign {left, right} tokens and return
                if (left_box[0] > right_box[0]):
                    left_box, right_box = right_box, left_box
                    left_token, right_token = right_token, left_token
                return left_token, left_box, right_token, right_box, True
    
            else:
                # no luck
                return None, None, None, None, False



