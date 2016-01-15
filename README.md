# Introduction
----------------
We are pleased to announce a new package for unmanned aerial vehicle (UAV) detection, an implementation basing on Chan-Vese model-based image segmentation. Its source code is available now.
 
The package aims to detect the UAV during UAV automonous take-off and landing. In another word, it is to locate the the center of the UAV in each frame. An initial closed curve is set according to previous detection results, and then Chan-Vese model-based segmentation algorithm is employed to detect object region. Finally, with the segmented object region, the UAV center is located.  
 
There are two cpp files in this package: ```main.cpp``` and ```cv_detection.cpp```. 

# System Requirements
------------------------
- Ubuntu 14.04
- ROS indigo

# Installation
<!-- ------------------------ -->
## with catkin
```
cd catkin_ws/src
git clone https://github.com/491734045/micros_cv_detection
cd ..
catkin_make
```


# Quick Start
------------------------
- Setup initial parameters in launch/CV_detection.launch
```
<param name="initial_x" type="int" value="320" />
<param name="initial_y" type="int" value="270" />
<param name="initial_r" type="int" value="20" />
```
The coordinate (initial_x, initial_y) is the center of the initial curve (circle) and initial_r is the radius ```
 
-**Open a new consol for video node**
- Run Video Node.

Note: This node pulishes video or continuous frames from camera as the input of CV_detection node.

-**Open a new consol for cv_detection node**.

Set up environment variables
```
source devel/setup.bash
```
Note: Run this command under `~/catkin_ws` directory. 
- Run detection Node
```
roslaunch cv_detection cv_detection.launch
```
- Demo

![image](https://cloud.githubusercontent.com/assets/11674154/9218708/384ea87e-4107-11e5-9685-ef04869a1113.png)
 

- The image is a single frame of the UAV autonomous landing video.
- The red circle represents initial curve.
- The green point is the UAV detection result.


Note: The package is inspired by and adapted from [1]. The details about the Chan-Vese model can be found in [2].

=======
- [1] D. Tang, T. Hu, L. Shen, D. Zhang and D. Zhou. Chan-Vese model based binocular visual object extraction for UAV autonomous take-off and landing. International Conference on Information Science and Technology, 2015, 67-73.
- [2] T. F. Chan and L. A. Vese. Active contours without edges. IEEE Transactions on Image Processing, 2001, 10: 266-277.
>>>>>>> origin/master
