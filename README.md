This is a container for an autonomous diver-following project. Deep object detection models are used for diver (and other objects such as ROV) detection. A simplified version of that is utilized for autonomous tracking (and following) of a (single) diver by an underwater robot. The ROS version, tested on Aqua-8 robot, is provided in the robo_chat_gest folder.

- Paper link:  https://ieeexplore.ieee.org/document/8461197
- Dataset collection information:  https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21837 
- A trained frozen model: provided in model_data folder

## Testing the detector
For testing individual images, run the [test_detector.py](test_detector.py) file. Change the image directory (im_dir) to test other images of interest.

| Single diver | Multiple divers | Divers and ROVs | 
|:--------------------|:--------------------|:----------------|
| ![det-7](/test_data/res/6.jpg) | ![det-7](/test_data/res/7.jpg)     | ![det-1](/test_data/res/1.jpg) |


## Testing the diver-tracker 
For testing on a diver-tracking video or sequences of images, run the [test_diver_tracker.py](test_diver_tracker.py) file. A couple of videos and image sequences are provided in the test_data folder. Change the argument values to test on other files.


### Video demo: https://youtu.be/cHQ9E-yRSho

| {Pic, Pic} | {Five, Two} | {Zero, Ok} | {Left, Left} | 
|:--------------------|:----------------|:----------------|:----------------
| ![det-86](/test_data/res/0.jpg)     | ![det-96](/test_data/res/5.jpg) |   ![det-106](/test_data/res/7.jpg) | ![det-1118](/test_data/res/10.jpg)     | 




## ROS version
- The robo_chat_gest folder contain the ROS-package version 
- This version is currently running on the Aqua MinneBot robot (more details: http://irvlab.cs.umn.edu)
- Feel free to cite the paper you find anything useful:  https://ieeexplore.ieee.org/document/8461197

```
@inproceedings{islam2018dynamic,
  title={{Dynamic Reconfiguration of Mission Parameters in Underwater Human-Robot Collaboration}},
  author={Islam, Md Jahidul and Ho, Marc and Sattar, Junaed},
  booktitle={{IEEE International Conference on Robotics and Automation (ICRA)}},
  pages={1--8},
  year={2018},
  organization={IEEE}
}

@article{islam2018understanding,
  title={{Understanding Human Motion and Gestures for Underwater Human-Robot collaboration}},
  author={Islam, Md Jahidul and Ho, Marc and Sattar, Junaed},
  journal={{Journal of Field Robotics (JFR)}},
  pages = {1--23},
  year={2018},
  publisher={Wiley Online Library}
}
```
