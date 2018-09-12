
Install 
    Python 2.7
        Opencv3
            python -m pip install opencv-python
    ROS
        ROS handles the communication between the different modules, transforms 3d-data and provides a handy 3d visualization for development
        Short introduction to ROS
            https://courses.cs.washington.edu/courses/cse466/11au/calendar/ros_cc_1_intro-jrsedit.pdf
        Install ROS kinetic for your specific OS
            http://wiki.ros.org/kinetic/Installation
            The ros-kinetic-desktop package is needed for our functions
    ardupilot
        Simulate a drone with the original firmware as Software in the Loop (SITL)
            http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html#clone-ardupilot-repository
    mavros-package
        To communicate with the SITL ardupilot in ROS
            1. Install: http://ardupilot.org/dev/docs/ros-install.html#installing-mavros
            2. Test: http://ardupilot.org/dev/docs/ros-sitl.html
    
Build/integrate src to ros framework
    create a catkin_workspace to compile the ros-specific code
        http://wiki.ros.org/catkin/Tutorials/create_a_workspace
    git clone
        this repo to catkin_ws/src/
    Compile/install  
        cd to the catkin_ws  
        catkin_make install
        source catkin_ws/devel/setup.bash

Run 
    Start simulated Drone 
        http://ardupilot.org/dev/docs/plane-sitlmavproxy-tutorial.html
        cd ardupilot/ArduPlane
        sim_vehicle.py --map --console
        wp load ../Tools/autotest/CMAC-circuit.txt
        mode auto
        arm throttle 
            ( if MAVPROXY window shows warning "APM: PreArm: AHRS not healthy Got MAVLink msg: COMMAND_ACK {command : 400, result : 4}" wait a few more seconds)
    mavros
        http://ardupilot.org/dev/docs/ros-sitl.html
        roslaunch apm.launch
    boatdetector
        change settings in launch/startBoatDetection.launch
            set videofile to play in videoReplaySettings (for example one of the prerecorded drone videos)
            set start and rewind 
        start detection
            rosrun searchwing publishFakeCameraCalib.py
        
        
Additional stuff
    /scripts folder
        generateROIproposalsPascalVoc.py
            generate ROIs for all the images in the provided folder in the pascal voc format alongside the images as xml files
            you can edit the labels by using the labelimg labeling tool
            https://github.com/tzutalin/labelImg
        extractDescriptorsFromROIs.py
            calculate descriptors for the provided groundtruth data and saves them to a hdf5 file 
        trainClassifier.py / trainClassifier.ipynb
            trains a random forest classifier based on the provided descriptors
        
        
    



