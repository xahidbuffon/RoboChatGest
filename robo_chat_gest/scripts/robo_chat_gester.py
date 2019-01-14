#! /usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/


Class for generating hand-gesture based instructions using robo_chat_gest
"""


# ros/python/opencv libraries and msgs
import sys
import os
import argparse
import cv2
import rospy
import roslib
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# local libraries
from gestureRecognizer import HandGestRecognition
from instructionGenerator import InstructionGeneration
from menueSelector import MenueSelection

# Aqua msgs
from aquacore.msg import Command
from ar_recog.msg import Tag, Tags



class RoboChatGest_pipeline:
        """ 
           Class for generating hand-gesture based instructions using robo_chat_gest 
        """
	def __init__(self, real_time=False):
                # instance for hand gesture recognition
		self.gest_rec = HandGestRecognition()
                # we have 10 classes (see the paper ieeexplore.ieee.org/document/8543168)
		self.classes = ['0', '1', '2', '3', '4', '5', 'left', 'right', 'pic','ok']
                self.obj_classes = { 0:'Zero',1:'One',2:'Two',3:'Three',4:'Four',5:'Five',6:'Left',7:'Right',8:'Pic',9:'Ok'}
                # instance for instruction generation
		self.ins = InstructionGeneration(self.classes)
                
                with open('/home/aqua/catkin_overlay_ws/src/robo_chat_gest/data/robo_chat_gest_params.yaml', 'r') as run_param_f:
                        run_param_dict = yaml.load(run_param_f)
		self.menue_mode = run_param_dict['set_Menue_mode_']
		self.robo_gest_mode = run_param_dict['set_RoboGest_mode_']
		self.bench_test = run_param_dict['set_Bench_Test_']
		self.publish_image = run_param_dict['set_Publish_Image_']
		self.use_single_hand = run_param_dict['use_Single_Hand_Gestures_only'] 

                # flags for Aqua menue selection
		self.men_sel = MenueSelection(self.classes, self.use_single_hand)
		self.menue_map = {'0':0, '1':1, '2':2, '3':3, '4':4, '5':5}
         
		#self.menue_mode = rospy.get_param('~set_Menue_mode_')
		#self.robo_gest_mode = rospy.get_param('~set_RoboGest_mode_')
		#self.bench_test = rospy.get_param('~set_Bench_Test_')
		#self.publish_image = rospy.get_param('~set_Publish_Image_')
		#self.use_single_hand = rospy.get_param('~use_Single_Hand_Gestures_only') 

		if real_time:
                        # settings only for real-time testing
                        # initialize rosnode and ros-opencv bridge for getting images
			rospy.init_node('gesture', anonymous=True)
			self.bridge = CvBridge()
                        # we use the back camera of Aqua for interaction
			self.topic_back_cam = '/camera_back/image_raw'
			image_back_cam = rospy.Subscriber(self.topic_back_cam, Image, self.imageCallBack, queue_size=10)

                        # this is the publisher for aqua tags (menue selection purpose)
			self.tags_pub = rospy.Publisher('aqua/tags', Tags, queue_size=10)
			self.stat_pub = rospy.Publisher('robo_chat_gest/status', String, queue_size=10)

			if self.publish_image:
                                # to visualize detection, publish this
				self.ProcessedRaw = rospy.Publisher('/robo_chat_gest/image', Image, queue_size=10)
			try:
				rospy.spin()
			except KeyboardInterrupt:
				print("Rospy Sping Shut down")

		else:
                        # settings only for bench-testing
			self.bench_test, self.publish_image = True, False
			



	def imageCallBack(self, back_im):
                """ 
                   CallBack function to get the image (from back camera) through the 
                    ros-opencv-bridge and start processing
                """
		try:
			self.original = self.bridge.imgmsg_to_cv2(back_im, "bgr8")
                        # *** Remember, the image is now BGR, have to convert to RGB
                        #      or the tensorflow model will suck 
		except CvBridgeError as e:
			print(e)
		if self.original is None:
			print ('frame dropped, skipping tracking')
		else:
                        # got the image, start processing
			self.ImageProcessor()




	def ImageProcessor(self):
                """ 
                   Process each frame
			> detect left and right hand-gestures
			> perform mapping to {left token, right token}s
			> use Finite state machine to generate full instruction
		   see more details in the paper: ieeexplore.ieee.org/document/8543168
                """
                # get the tokens (and the bounding boxes for vizualization)
		left_token, left_box, right_token, right_box, success_ = self.gest_rec.Get_gest(self.original, self.use_single_hand)
		print ("Hand gestures detection success: {2}. token: ({0}, {1})".format(right_token, left_token, success_))

		if success_:
			# ROBO_GEST mode
			if self.robo_gest_mode:
                                # reverse left and right since camera(left, right) == person(right, left)
                                #  then pass it to generate instruction
				get_token, done_ = self.ins.decode(right_token, left_token)
				print (get_token, done_)
				if done_:
                                        print 
                                        print ("*** Decoded Instruction: {0}".format(get_token))
                                        print


			# For Menue Selection only
			if self.menue_mode:
			        #Notifies aquamenu that the gesture is waiting.
			        msg = String()
			        msg.data = 'WAITING'
			        self.stat_pub.publish(msg)

				men_ins_, men_done_ = self.men_sel.decode(right_token, left_token)
                                #print(men_ins_, men_done_)
				if men_done_:
                                        #Notifies aquamenu that the gesture is done.
					msg.data = 'DONE'
					self.stat_pub.publish(msg)
                                        print 
                                        print ("Decoded Instruction: {0}".format(men_ins_))
                                        print
					men_tok = men_ins_.split(' ')
					if (len(men_tok)>0 and men_tok[1] in self.menue_map.keys()):
						menue_selected = self.menue_map[men_tok[1]]
						msg = Tags()
						tag = Tag()
						tag.id = menue_selected
						msg.tags = [tag]
						self.tags_pub.publish(msg)
						print ('***** Menue selected :: {0}'.format(menue_selected))
                                                print

			if (self.bench_test or self.publish_image):
				localised_objs = [(left_token, left_box), (right_token, right_box)]
				self.draw_boxes_and_labels(localised_objs)
		


		if self.bench_test:
			self.showFrame(self.original, 'test_viz')

		if self.publish_image:
			msg_frame = CvBridge().cv2_to_imgmsg(self.original, encoding="bgr8")
			self.ProcessedRaw.publish(msg_frame)



	def draw_boxes_and_labels(self, localised_objs, box_color=(0, 255, 255)):
		"""
		   vizializer that draws boxes and labels on the localized objects
			> localized_objs: list of tupples (class id, box)  
		"""
		for (i, bbox_cv2) in localised_objs:
			# Draw the object boxes
			left, right, top, bottom = bbox_cv2[0], bbox_cv2[1], bbox_cv2[2], bbox_cv2[3]
			cv2.rectangle(self.original, (left, top), (right, bottom), box_color, 4)
			# Draw the labels
			top2 = 0 if top<40 else top-40
                        cv2.rectangle(self.original, (left, top2), (right, top), box_color, -1, 1)
			cv2.putText(self.original, self.obj_classes[i], (left, top2+20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3)

		

	##########################################################################
	###   For bench testing with dataset images ###############################
	def showFrame(self, frame, name):
		cv2.imshow(name, frame)
		cv2.waitKey(1000)

	# stream images from directory Dir_
	def image_streamimg(self, Dir_):
		from eval_utils import filter_dir
		dirFiles = filter_dir(os.listdir(Dir_))
		for filename in dirFiles:
			self.original = cv2.imread(Dir_+filename)
			self.ImageProcessor()
	####################################################################################
