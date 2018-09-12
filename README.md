# CV for searchwing drones
![alt text](https://raw.githubusercontent.com/julled/searchwing/master/screenshot.jpg)
## Install dependencies
### Python 2.7  
* Opencv3 
```
python -m pip install opencv-python
```
### ROS
ROS handles the communication between the different modules, transforms 3d-data and provides a handy 3d visualization for development
* [Short introduction to ROS](https://courses.cs.washington.edu/courses/cse466/11au/calendar/ros_cc_1_intro-jrsedit.pdf)
* Install ros-kinetic-desktop for your specific OS: http://wiki.ros.org/kinetic/Installation

### ardupilot
Simulate a drone with the original firmware as Software in the Loop (SITL)
* Install: http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#clone-ardupilot-repository
### mavros-package
To communicate with the SITL ardupilot in ROS
* Install: http://ardupilot.org/dev/docs/ros-install.html#installing-mavros
* Test: http://ardupilot.org/dev/docs/ros-sitl.html
    
## Build/integrate src to ros framework
* create a catkin_workspace to compile the ros-specific code: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
* git clone this repo to 
```
/catkin_ws/src/
```
* Compile/install  
``` 
cd /catkin_ws  
catkin_make install
source /catkin_ws/devel/setup.bash
```
## Run 
1. Start simulated Drone: http://ardupilot.org/dev/docs/plane-sitlmavproxy-tutorial.html
```
cd ardupilot/ArduPlane
sim_vehicle.py --map --console
wp load ../Tools/autotest/CMAC-circuit.txt
mode auto
arm throttle 
```
if MAVPROXY window shows warning 
```
APM: PreArm: AHRS not healthy Got MAVLink msg: COMMAND_ACK {command : 400, result : 4}
```
wait a few more seconds.

2. mavros: http://ardupilot.org/dev/docs/ros-sitl.html
```
cd /ardupilot_ws/launch
roslaunch apm.launch
```
3. boatdetector: 
* change settings in launch/startBoatDetection.launch:
    * set videofile to play in videoReplaySettings (for example one of the prerecorded drone videos)
    * set start and rewind 
* start detection
```
rosrun searchwing publishFakeCameraCalib.py
```
Now you should see the detections in the rviz programm

     
## Additional stuff
### /scripts folder
* generateROIproposalsPascalVoc.py
    * generate ROIs for all the images in the provided folder in the pascal voc format alongside the images as xml files
    * you can edit the labelfiles (xmls) by using the labelImg labeling tool: https://github.com/tzutalin/labelImg
* extractDescriptorsFromROIs.py
    * calculate descriptors for the provided groundtruth data and saves them to a hdf5 file 
* trainClassifier.py / trainClassifier.ipynb
    * trains a random forest classifier based on the provided descriptors
        
        
    



