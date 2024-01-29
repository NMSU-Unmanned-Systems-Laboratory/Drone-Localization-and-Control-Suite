
# Drone Localization and Control Suite

  
This repository contains a python library to support Unmanned Aircraft System control and localization in an abstracted and streamlined interface. It allows to receive global localization positioning and orientation of UASs from a motion capture system and the delivering of real-time control signals to the UAS system based on the global positioning. 
**<br>**

The requirements for this library are as follows:
- A Linux system installed with ROS Noetic Ninjemys
- An Optitrack Motion Capture system that publishes ROS nav_msgs/Odometry messages to a ROS topic which represents the UAS global position. (achieved through a [ROS driver](https://wiki.ros.org/mocap_optitrack 'Optitrack ROS driver'))
- A UAS which is able to be tracked using the global localization system and send control signals to. (Currently support DJI Tello and Parrot Bebop 1/2)

If you would like documentation on the classes provided in this python library, skip to the 'Documentation' section below.

## Installation
Before the library is able to communicate properly, its dependencies first need to be installed. These include ROS and its related drivers. The process for all installations are listed below:
**<br>**

#### ROS Noetic Installation
The Directions for installing ROS can be found [here](https://wiki.ros.org/noetic/Installation 'https://wiki.ros.org/noetic/Installation'). The system was tested on a Linux system running Ubuntu 20.04 Focal Fossa
**<br>**

#### Optitrack ROS Driver Installation
To download the Optitrack ROS driver, simply run the below command in the terminal after installing ROS:
```
sudo apt-get install ros-noetic-mocap-optitrack
```
After installing the driver, its settings can be configured through the configuration file. The file can be found by executing the following commands
```
roscd mocap_optitrack
cd config
gedit mocap.yaml
```
With this file open you should see .yaml formatting that looks like this 
```
mocap_node:
    ros__parameters:
        rigid_bodies:
            1:
                pose: "Robot_1/pose"
                pose2d: "Robot_1/ground_pose"
                odom: Robot_1/Odom
                tf: tf
                child_frame_id: "Robot_1/base_link"
                parent_frame_id: "world"
            2:
                pose: "Robot_2/pose"
                pose2d: "Robot_2/ground_pose"
                odom: Robot_2/Odom
                tf: tf
                child_frame_id: "Robot_2/base_link"
                parent_frame_id: "world"
                
        optitrack_config:
                multicast_address: "224.0.0.1"
                command_port: 1510
                data_port: 9000
                enable_optitrack: true
```
Make sure that the parameters under the 'optitrack_config' section match the multicast address, command port, and data port of your Optitrack system.

The sections labeled with integers represent rigid bodies in the Optitrack system. The integer represents the streaming ID of the rigid body in the Optitrack Motive software. The pose, pose2d, and odom fields will designate the ROS topics for their respective data.

For example, if I wanted to create a configuration that only supports a rigid body with streaming ID of 12 and a name of Drone_03, the mocap.config file would look like this:
```
mocap_node:
    ros__parameters:
        rigid_bodies:
            12:
                pose: "Drone_03/pose"
                pose2d: "Drone_03/ground_pose"
                odom: Drone_03/Odom
                tf: tf
                child_frame_id: "Drone_03/base_link"
                parent_frame_id: "world"
                
        optitrack_config:
                multicast_address: "224.0.0.1"
                command_port: 1510
                data_port: 9000
                enable_optitrack: true
```

To add another rigid body, simply add a new section with its streaming ID and name:
```
mocap_node:
    ros__parameters:
        rigid_bodies:
            12:
                pose: "Drone_03/pose"
                pose2d: "Drone_03/ground_pose"
                odom: Drone_03/Odom
                tf: tf
                child_frame_id: "Drone_03/base_link"
                parent_frame_id: "world"
           13:
                pose: "Drone_04/pose"
                pose2d: "Drone_04/ground_pose"
                odom: Drone_04/Odom
                tf: tf
                child_frame_id: "Drone_04/base_link"
                parent_frame_id: "world"
                
        optitrack_config:
                multicast_address: "224.0.0.1"
                command_port: 1510
                data_port: 9000
                enable_optitrack: true
```

After the configuration file is set and the data is streaming from the Optitrack system, it is ready to work with the library
**<br>**

####  Drone Interface
Depending on if you are working with a Parrot Bebop1/2 or DJI Tello, the interface will be different. The interface with the DJI Tello and the Bebop drones is integrated in python with the djitellopy and pyparrot modules respectively, and will be downloaded alongside this library. If however you will be configuring the control of the Parrot Bebop through the Bebop ROS driver [bebop_autonomy](https://bebop-autonomy.readthedocs.io/en/latest/ 'https://bebop-autonomy.readthedocs.io/en/latest/'), then some additional steps are required. ==NOTE==: This process has been recorded as unstable, and may or may not work for your system. To improve performance and reliability, it is recommended to work with the python libraries and skip to the 'Drone Localization and Control Suite Installation' section.

Make  a workspace for the Bebop driver:
```
mkdir -p ~/Workspaces/bebop_workspace/src

cd ~/Workspaces/bebop_workspace

catkin_make
```
Install parrot arsdk into workspace:
```
cd ~/Workspaces/bebop_workspace/src

git clone https://github.com/antonellabarisic/parrot_arsdk.git

cd parrot_arsdk

git checkout noetic_dev

sudo ln -s /usr/bin/python3 /usr/bin/python

cd ~/Workspaces/bebop_workspace

catkin_make
```
Install bebop_autonomy into workspace:
```
cd ~/Workspaces/bebop_workspace/src

git clone https://github.com/autonomylab/bebop_autonomy.git

gedit bebop_autonomy/bebop_driver/src/bebop_video_decoder.cpp
```

In 'bebop_video_decoder.cpp' Modify:
- line 93: replace CODEC_CAP_TRUNCATED with AV_CODEC_CAP_TRUNCATED
- line 95: replace CODEC_FLAG_TRUNCATED with AV_CODEC_FLAG_TRUNCATED
- line 97: replace CODEC_FLAG2_CHUNKS with AV_CODEC_FLAG2_CHUNKS
- Save and Exit
 
Run the following commands to to complete the driver installation:
```
echo export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/usr/Workspaces/bebop_workspace/devel/lib/parrot_arsdk/ >> ~/.bashrc

cd ~/Workspaces/bebop_workspace

catkin_make

catkin_make
```

If the build fails, then there was an instability at some point in the installation process. (ie. All hell has broke loose and its every man for himself).


**<br>**

####  Drone Localization and Control Suite Installation
To install the Python library included in this repository, simply take the wheel file provided in its root and install it using pip in your desired environment.

from the root of the repository (where '\*.\*.*' is the version of the library):
```
pip install Drone_Loc_and_Cont_Suite-*.*.*.whl
```

This will install the library in your current active python environment. You are now ready to use the Drone Localization and Control Suite!



