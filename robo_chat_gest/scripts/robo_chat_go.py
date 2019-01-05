#!/usr/bin/env python

"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/


Hand gesture based programming using robo_chat_gest
>> relevant papers: 
    >> https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21837
    >> https://ieeexplore.ieee.org/document/8461197
> how to run:
	> launch: roslaunch robo_chat_gest robo_chat_xahid.launch
	> rosrun: rosrun robo_chat_gest robo_chat_go.py
"""

# ros/python/opencv libraries
import rospy 
import sys
import argparse
# local libraries
from robo_chat_gester import RoboChatGest_pipeline



# only when running in Aqua
go = RoboChatGest_pipeline(real_time=True)



#-----------------------------------------------------------------------------------------------------------------------------
# for bench test only
# ** not checked for a while
"""
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--im_dir', required=False, dest='im_dir', type=str, default=None, help='Full path of image sequences')
    args = parser.parse_args()
    if (args.im_dir is not None):
        go = RoboChatGest_pipeline()
        go.image_streamimg(args.im_dir)
    else:
    	go = RoboChatGest_pipeline(real_time=True)
"""

