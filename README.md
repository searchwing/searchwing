# CV for searchwing drones
![alt text](https://raw.githubusercontent.com/julled/searchwing/master/screenshot.jpg)
## Install dependencies
### Python 2.7  
Install Opencv3 for python 2.7:
```
python -m pip install opencv-python
```
### ROS
ROS handles the communication between the different modules, transforms 3d-data and provides a handy 3d visualization for development
* [Short introduction to ROS](https://courses.cs.washington.edu/courses/cse466/11au/calendar/ros_cc_1_intro-jrsedit.pdf)
* Install ros-kinetic-desktop for your specific OS: http://wiki.ros.org/kinetic/Installation

## Build/integrate src to ros framework
* create a catkin_workspace to compile the ros-specific code: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
* git clone this repo  
```
cd /catkin_ws/src/
git clone https://github.com/julled/searchwing.git
```
* Compile/install  
``` 
cd /catkin_ws  
catkin_make install
source /catkin_ws/devel/setup.bash
```
* Download dataset:
write me to get access to the bodensee-dataset.

## Run "detection by tracking" algorithm on the bodensee-dataset
* New Terminal: Start roscore
```
roscore
```

* New Terminal: Start dataset playbag.
```
cd datasetDownloadPath
rosbag play bodenSee2018HalfRes.bag --clock --start=0 --rate=2
```

* New Terminal: 
Setup terminal
```
source /catkin_ws/devel/setup.bash
```
Tell ROS to the provided timestamps from the dataset for the internal clock
```
rosparam set use_sim_time true
```
start boatdetector
```
roslaunch searchwing bodenseeDataset.launch
```
Now you should see the detections in rviz like in the screenshot above.


If you want to edit/start/debug the boadDetectorNode.py with a editor, please ensure you run
```
rosparam set use_sim_time true
```
before you start the editor in the terminal. Also comment out the start of the node in the launch file.

     
## Additional stuff
### /scripts folder
* generateROIproposalsPascalVoc.py
    * generate ROIs for all the images in the provided folder in the pascal voc format alongside the images as xml files
    * you can edit the labelfiles (xmls) by using the labelImg labeling tool: https://github.com/tzutalin/labelImg
* extractDescriptorsFromROIs.py
    * calculate descriptors for the provided groundtruth data and saves them to a hdf5 file 
* trainClassifier.py / trainClassifier.ipynb
    * trains a random forest classifier based on the provided descriptors
        
        
    



