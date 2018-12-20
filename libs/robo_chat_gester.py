#! /usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/


Class for generating hand-gesture based instructions using robo_chat_gest
"""


import sys
import os
import argparse
import cv2
import yaml

# local libraries
from gestureRecognizer import HandGestRecognition
from instructionGenerator import InstructionGeneration
from menueSelector import MenueSelection


class RoboChatGest_pipeline:
        """ 
           Class for generating hand-gesture based instructions using robo_chat_gest 
        """
	def __init__(self, real_time=False):
                # instance for hand gesture recognition
		self.gest_rec = HandGestRecognition()
                # we have 10 classes (see the paper ieeexplore.ieee.org/document/8543168)
		self.classes = ['0', '1', '2', '3', '4', '5', 'left', 'right', 'pic','ok']
                # instance for instruction generation
		self.ins = InstructionGeneration(self.classes)
                # flags for Aqua menue selection
		self.men_sel = MenueSelection(self.classes)
		self.menue_map = {'0':0, '1':1, '2':2, '3':3, '4':4, '5':5}

		self.frame_no = 0
                
                with open('model_data/robo_chat_gest_params.yaml', 'r') as run_param_f:
                        run_param_dict = yaml.load(run_param_f)
		self.menue_mode = run_param_dict['set_Menue_mode_']
		self.robo_gest_mode = run_param_dict['set_RoboGest_mode_']
		self.bench_test = run_param_dict['set_Bench_Test_']
		self.publish_image = run_param_dict['set_Publish_Image_']
		self.use_single_hand = run_param_dict['use_Single_Hand_Gestures_only'] 
			

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
				men_ins_, men_done_ = self.men_sel.decode(right_token, left_token)
                                #print(men_ins_, men_done_)
				if men_done_:
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
		




