#!/usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/
Any part of this repo can be used for academic and educational purposes only
"""

import cv2
import os
import argparse

# local libraries
from libs.robo_chat_gester import RoboChatGest_pipeline
from libs.utils import draw_boxes_and_labels, check_file_ext


if __name__ == '__main__':
    """ for testing RoboChatGest
            > use argument --test_vid to test video or sequence of images
        other arguments:
            --im_dir >> path of image folder
            --vid    >> path of the test video file (0 for webcam)
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--im_dir', required=False, dest='im_dir', type=str, default='test_data/ins_test/hover', help='Folder containing images')
    parser.add_argument('--vid', required=False, dest='vid', type=str, default='', help='Video file')
    parser.add_argument('--test_vid', required=False, dest='test_vid', type=bool, default=False, help='Test video or images')
    parser.add_argument('--menu_mode', required=False, dest='menu_mode', type=bool, default=False, help='menu_mode or robo_chat_gest mode')
    args = parser.parse_args()

    robochat_gest = RoboChatGest_pipeline(args.menu_mode)

    i=0
    if not args.test_vid:
        # test a sequence of images
        IMAGE_PATHS = [os.path.join(args.im_dir, f) for f in os.listdir(args.im_dir) if check_file_ext(f)]
        IMAGE_PATHS.sort(key=lambda f: int(filter(str.isdigit, f)))
        for im_file in IMAGE_PATHS:
            frame = cv2.imread(im_file)
            frame = robochat_gest.ImageProcessor(frame, vizualize=True, wait_time=1000) 
                      
    else:
        # test on a video file 
        counter=0   
        """try to use the webcam if vid is not provided. should detect gestures with
             reasonable accuracy although the model is only trained on underwater images!
        """
        vid_src = 0 if args.vid=='' else args.vid
        cap = cv2.VideoCapture(vid_src)
        while(cap.isOpened()):
            ret, frame = cap.read()
            if frame is not None:
                frame = robochat_gest.ImageProcessor(frame, vizualize=True, wait_time=50) 

            else:
                counter += 1
                if counter > 10: break

        cap.release()












