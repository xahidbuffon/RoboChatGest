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

# local libraries
from gestureRecognizer import HandGestRecognition
from instructionGenerator import InstructionGeneration
from menueSelector import MenueSelection
from utils import draw_boxes_and_labels, write_instructions

class RoboChatGest_pipeline:
        """ 
           Class for generating hand-gesture based instructions using robo_chat_gest 
        """
	def __init__(self, menue_mode=False):
                # instance for hand gesture recognition
		self.gest_rec = HandGestRecognition()
                # we have 10 classes (see the paper ieeexplore.ieee.org/document/8543168)
		self.classes = ['0', '1', '2', '3', '4', '5', 'left', 'right', 'pic','ok']
                self.obj_classes = { 0:'Zero',1:'One',2:'Two',3:'Three',4:'Four',5:'Five',6:'Left',7:'Right',8:'Pic',9:'Ok'}
                # instance for instruction generation
		self.ins = InstructionGeneration(self.classes)
                # flags for Aqua menue selection
		self.men_sel = MenueSelection(self.classes)

                if menue_mode: 
			self.menue_mode, self.robo_gest_mode = True, False
		else:
			self.menue_mode, self.robo_gest_mode = False, True    
		self.use_single_hand = False
			

	def ImageProcessor(self, img, vizualize=False, wait_time=1):
                """ 
                   Process each frame
			> detect left and right hand-gestures
			> perform mapping to {left token, right token}s
			> use Finite state machine to generate full instruction
		   see more details in the paper: ieeexplore.ieee.org/document/8543168
                """
                # get the tokens (and the bounding boxes for vizualization)
		left_token, left_box, right_token, right_box, success_ = self.gest_rec.Get_gest(img, self.use_single_hand)
		print ("Hand gestures detected: ({0}, {1})".format(right_token, left_token))

		if success_:
                        if vizualize:
                        	localised_objs = [(left_token, left_box), (right_token, right_box)]
                        	img = draw_boxes_and_labels(img, localised_objs, self.obj_classes)
			
			if self.robo_gest_mode: # ROBO_GEST mode
                                # camera(left, right) == person(right, left)
				get_token, done_ = self.ins.decode(right_token, left_token)
			elif self.menue_mode: # Menue Selection mode
				get_token, done_ = self.men_sel.decode(right_token, left_token)
                        else: pass

                        if (not get_token or get_token != ''): 
	                        img = write_instructions(img, get_token)
				if done_:
					#>> Here we perform the operations/service calls to change the menue
                                        print 
                                        print ("Decoded Instruction: {0}".format(get_token))
                                        print
					
		self.vizualize_img(img, vizualize, wait_time)
                return img



	def vizualize_img(self, img, vizualize=False, wait_time=1):
                """ 
                   Annotates bounding box, label and then shows it
                """
		if vizualize:
			cv2.imshow('Annotated Image', img)
			cv2.waitKey(wait_time)
		




