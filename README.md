This is a container for a hand gesture-based human-robot communication framework named RoboChatGest. This allows divers to use a set of simple hand gestures to communicate instructions to an underwater robot and dynamically reconfigure program parameters during a mission. The ROS version, tested on Aqua-8 robot, is provided in the robo_chat_gest folder.

- Model and dataset information:  https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21837 
- RoboChatGest programming rules:  https://ieeexplore.ieee.org/document/8461197
- A trained frozen model: provided in model_data folder

### Hand gestures 
The following set of 10 simple and intuitive hand gestures are used:

| zero | one | two | three | four | five | left | right | pic | ok | 
|:------|:------|:------|:------|:------|:------|:------|:------|:------|:------|
| ![det-1](/test_data/res/d0.jpg) | ![det-5](/test_data/res/d1.jpg)     | ![det-9](/test_data/res/d2.jpg) | ![det-13](/test_data/res/d3.jpg) | ![det-17](/test_data/res/d4.jpg)     | ![det-2](/test_data/res/d5.jpg) | ![det-6](/test_data/res/d6.jpg) | ![det-10](/test_data/res/d7.jpg)     | ![det-14](/test_data/res/d8.jpg) |![det-18](/test_data/res/d9.jpg) |
| ![det-3](/test_data/res/u0.jpg) | ![det-7](/test_data/res/u1.jpg)     | ![det-11](/test_data/res/u2.jpg) | ![det-15](/test_data/res/u3.jpg) | ![det-19](/test_data/res/u4.jpg)     | ![det-4](/test_data/res/u5.jpg) | ![det-8](/test_data/res/u6.jpg) | ![det-12](/test_data/res/u7.jpg)     | ![det-16](/test_data/res/u8.jpg) |![det-20](/test_data/res/u9.jpg) |



### Testing the detector
Use the [test_detector.py](test_detector.py) file to test other images or video files of interest.


| {pic, pic} | {five, two} | {zero, ok} | {left, left}
|:--------------------|:----------------|:----------------|:----------------
| ![det-21](/test_data/res/0.jpg)     | ![det-22](/test_data/res/5.jpg) |   ![det-23](/test_data/res/7.jpg) |  ![det-23](/test_data/res/10.jpg) | 
#### Demos: https://youtu.be/cHQ9E-yRSho


### Testing the RoboChatGest 
In RoboChatGest, the sequence of hand gestures are used to generate instructions for:
- Task switching: STOP current task and SWITCH to another (predefined) task
- Parameter reconfiguration: CONTD current program, but UPDATE values of a (predefined) parameter

For instance, instructing the robot to 'STOP current task and HOVER' can be done as follows:
- Start token for STOP current task {0, 0} + HOVER token {5, 5} + confirmation token {ok, ok}
- Hence, {left-hand, right-hand} gesture tokens = {0, 0}, {5, 5}, {ok, ok} 

| RoboChatGest mode | STOP HOVER | Token: STOP HOVER | Token: STOP HOVER GO |
|:--------------------|:----------------|:----------------|:----------------
| ![det-24](/test_data/res/r1.jpg) | ![det-24](/test_data/res/r3.jpg)     | ![det-25](/test_data/res/r7.jpg) |   ![det-26](/test_data/res/r11.jpg) | 

Details about the hand gestures-to-instruction mapping can be found in the paper. We keep chanding these mapping rules based on specific application requirements; we use a simple Finite-State Machine (FSM) to implement a mapping. See [instructionGenerator.py](/libs/instructionGenerator.py) for details. We also use a different FSM for menue selection, i.e., for switching between five menue options in the Aqua robot (see [menueSelector.py](/libs/menueSelector.py) for details); to select a menue, the {left-hand, right-hand} gesture tokens are: {ok, ok}, {menue #, menue #}. For example: 

| Menue mode | Token: SELECT MENUE | Token: SELECT MENUE | Token: SELECT MENUE 3 |
|:--------------------|:----------------|:----------------|:----------------
| ![det-24](/test_data/res/m1.jpg) | ![det-24](/test_data/res/m3.jpg)     | ![det-25](/test_data/res/m5.jpg) |   ![det-26](/test_data/res/m10.jpg) | 



#### Demos: https://youtu.be/evPMcn_YhhY, https://youtu.be/An4IdMV_VtU



### ROS version
- The robo_chat_gest folder contain the ROS-package version 
- This version is currently running on the Aqua MinneBot robot (more details: http://irvlab.cs.umn.edu)
- Feel free to cite the papers if you find anything useful!

```
@article{islam2018understanding,
  title={{Understanding Human Motion and Gestures for Underwater Human-Robot collaboration}},
  author={Islam, Md Jahidul and Ho, Marc and Sattar, Junaed},
  journal={{Journal of Field Robotics (JFR)}},
  pages = {1--23},
  year={2018},
  publisher={Wiley Online Library}
}

@inproceedings{islam2018dynamic,
  title={{Dynamic Reconfiguration of Mission Parameters in Underwater Human-Robot Collaboration}},
  author={Islam, Md Jahidul and Ho, Marc and Sattar, Junaed},
  booktitle={{IEEE International Conference on Robotics and Automation (ICRA)}},
  pages={1--8},
  year={2018},
  organization={IEEE}
}
```


