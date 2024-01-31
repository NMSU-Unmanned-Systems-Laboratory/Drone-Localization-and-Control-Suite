
# Drone Localization and Control Suite

  
This repository contains a python library to support Unmanned Aircraft System control and localization in an abstracted and streamlined interface. It allows to receive global localization positioning and orientation of UASs from a motion capture system and the delivering of real-time control signals to the UAS system based on the global positioning. 
<br>

The requirements for this library are as follows:
- A Linux system installed with ROS Noetic Ninjemys
- An Optitrack Motion Capture system that publishes ROS nav_msgs/Odometry messages to a ROS topic which represents the UAS global position. (achieved through a [ROS driver](https://wiki.ros.org/mocap_optitrack 'Optitrack ROS driver'))
- A UAS which is able to be tracked using the global localization system and send control signals to. (Currently support DJI Tello and Parrot Bebop 1/2)

If you would like documentation on the classes provided in this python library, skip to the 'Documentation' section below.

## Installation
Before the library is able to communicate properly, its dependencies first need to be installed. These include ROS and its related drivers. The process for all installations are listed below:
<br>

### ROS Noetic Installation
The Directions for installing ROS can be found [here](https://wiki.ros.org/noetic/Installation 'https://wiki.ros.org/noetic/Installation'). The system was tested on a Linux system running Ubuntu 20.04 Focal Fossa
<br>

### Optitrack ROS Driver Installation
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
<br>

###  Drone Interface
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
<br>

###  Drone Localization and Control Suite Installation
To install the Python library included in this repository, simply take the wheel file provided in its root and install it using pip in your desired environment.

from the root of the repository (where '\*.\*.*' is the version of the library):
```
pip install Drone_Loc_and_Cont_Suite-*.*.*.whl
```

This will install the library in your current active python environment. You are now ready to use the Drone Localization and Control Suite!
<br>

## Documentation

Drone Localization and Control Suite provides two main classes for the user to interact with: Controller, and Drone. The combination allows ease of heterogeneous combinations of Unmanned Aerial System brands, types, and controllers. The classes, their attributes, and some examples are explained below:
<br>

### Drone Class
The Drone class provides a customizable and abstracted interface with sending control to a UAS. This includes setting up communication with, sending signals to takeoff, land, and control, and tracking position of the UAS. The object can be connected with a Controller class to add another layer of complexity to the control sent to the UAS.

**Attributes**
- bool **procedureRun**: A classwide variable to control the start and ending of experiements
- str **name**: The unique name for the drone
- int **type**: The type of drone for communication and control purposes
- Controller **controller**: The Controller object to be used when sending control signals
- function **callbackFunc**: The callback function that is used when localization data is received from the motion capture system
- list[] **bounds**: If working in a restrained area, the box to contain all control signals within
<br>

**Functions**

- **Drone(name:str, type:str, controller:Controller  ==  None, callback, bounds  =  None, ip  =  None)**: 
Constructor for the drone class <br>
	- Parameters:
		- bool **procedureRun**: A classwide variable to control the start and ending of experiements
		- str **name**: The unique name for the drone
		- str **type**: The type of drone for communication and control purposes
		- Controller **controller**: The Controller object to be used when sending control signals
		- function **callbackFunc**: The callback function that is used when localization data is received from the motion capture system
		- list[] **bounds**: If working in a restrained area, the box to contain all control signals within. Should be formatted with a list of 2D tuples representing lower and upper bounds for each degree of freedom provided to Controller parameter.

	Returns: Object of type Drone
<br>
- **set_controller_points(self, setPoints:list, reset  =  True)** :
Sets the setpoints of the drones controllers. Taking into account the bounds of the Drone object.<br>
	- Parameters:
		- list **setPoints**: List of new set-points for each degree of freedom
		- bool **reset**: A boolean to represent whether to reset the cache of the controllers with the new set-points
<br><br>
- **send_cont(pos:list, rotPosConts:bool  =  True, euler:bool  =  True, rotAxis:str  =  'yaw', verbose  =  False, raw_conts  =  None)** :
Sends global control signals in accordance with a given position, and the controller/setpoint. Account for necessary rotations if desired.<br>
	- Parameters:
		- list[float] **pos**: Positional control signal vector. One value for each positional degree of freedom
		- bool **rotPosConts**: Rotate the given control vector to the global frame (Default = True)
		- bool **euler**: If the orientation of the position is in Euler radians. (Default = True)
		- str **rotAxis**: Axis of rotation (default = 'yaw')
		- bool **verbose**: if True, will print the control signals sent to the drone
		- list[float] **raw_conts**: If not None, will send the raw control values of the parameter directly to the drone. Must be of size numPosAxis + numRotAxis
<br><br>
- **stop_movement()** :
Sends a command of zero control to the Drone
<br><br>
- **land(safe=True)** :
Sends a command of zero control to the Drone.<br>
	- Parameters:
		- bool **safe**: If available, executes a safe land command when True.
<br><br>
- **takeoff(self, safe=True)** :
Executes command to takeoff the drone.<br>
	- Parameters:
		- bool **safe**: If available, executes a safe takeoff command when True.
<br><br>
- **takeoff_drones(drones:list)** :
Executes takeoff command on every drone object in list parameter.<br>
	- Parameters:
		- list **drones**: list of Drone objects to execute the commands on.
<br><br>
- **stop_and_land_drones(drones:list)** :
Executes the stop_movement and land commands on every drone object in list parameter.<br>
	- Parameters:
		- bool **safe**: If available, executes a safe takeoff command when True.
<br><br>
- **disconnect()** :
For use with pyparrot and Tello drones. Ends the communication and disconnects the drone.
<br><br>
- **at_setpoint(pos, admittedErrs)**: 
Indicates whether the current position of the drone is within a permitted range to the drone's setpoint.<br>
	- Parameters:
		- list[float] **pos**: position along every degree of freedom
		- list[float] **admittedErrs**: list of absolute error allowances for each degree of freedom to assume the drone has reached its setpoint

	Returns: bool that, when True, indicates the given position has reached the Drone's current setpoints
<br>
- **create_tello_swarm()** :
Call this after creating all your Drone objects of type 'tello'. This encapsulates the tello drone objects into a tello swarm to execute commands to the tello drones simultaneously.
<br><br>

### Controller Class

The Controller class is a customizable controller to use with Drone objects, and give command values to the Drone object based on the controller type. Currently the class only supports pid control through the interface, but external controllers can still be used through the Drone class.
<br>

**Attributes**

- str type: a string to designate the type of controller
- list[] conts: A list of controllers for every degree of freedom
- list[float] numPosAxis: The number of positional degrees of freedom to control, up to 3
- list[float] numPosRot: The number of rotational degrees of freedom to control, up to 3
<br>

**Functions**

-  **Controller(type:str, numPosAxis:int, numRotAxis:int, contParams:list, setPoints  =  None)**:
	Constructor for the Controller class <br>

	- Parameters:
		- str **type**: The type of Controller to use when calculating control values
		- int **numPosAxis**: The number of positional degrees of freedom to control using the controller (must be a value from 1 to 3)
		- int **numRotAxis**: The number of rotational degrees of freedom to control using the controller (must be a value from 1 to 3)
		- list[float] **setPoints**: If not None, sets contains a list of setpoints for each degree of freedom that are set during instantiation. (default = None)

	Returns: Object of type Controller
<br>

-  **rotation_goal_standardize(self, goal, out  =  True)**:
	Transforms any amount of degrees to the degree or radian range of -180 to 180 <br>

	- Parameters:
		- float **goal**: The angle in degrees to convert to the range -180 to 180
		- bool **out**: Signifies whether to return RAD or Degrees. (default: True)
	True represents a return value in Radians
	False represents a return value in Degrees

	Returns: A float in the degree or radian range of -180 to 180
<br>


-  **rotation_goal_standardize(goal, out  =  True)**:
	Transforms any amount of degrees to the degree or radian range of -180 to 180 <br>

	- Parameters:
		- float **goal**: The angle in degrees to convert to the range -180 to 180
		- bool **out**: Signifies whether to return RAD or Degrees. (default: True)
	True represents a return value in Radians
	False represents a return value in Degrees

	Returns: A float in the degree or radian range of -180 to 180
<br>

-  **set_points(setPoints:list, reset  =  True):**:
	Sets the set-points of all internal degree of freedom controllers to the points given <br>

	- Parameters:
		- list **setPoints**: List of new set-points for each degree of freedom
		- bool **reset**: A boolean to represent whether to reset the cache of the controllers with the new set-points
<br>

-  **get_optimal_rot_pos(rot_curr, goal):**:
	Given a current rad position and a goal rad position in the range -pi to pi, calculate whether it is shorter to travel to the goal via positive or negative rotation <br>

	- Parameters:
		- float **rot_curr**: current rotational position in radians
		- float **goal**: rotational goal in radians

	Returns: float representing the new current rotational position that corresponds to the shortest path to reach the goal

	ex1: given a position of 175 degrees and a goal of -175 degrees, the function would return -185 as an altered current rotational position

	ex2: given a position of 23 degrees and a goal of 54 degrees, the function would return 23 as the current rotational position
<br>

-  **get_out(pos:list):**:
	Gives outs of the controller for each axis given current position<br>

	- Parameters:
		- list **pos**: current position of controlled device for each degree of freedom

	Returns: list output of each controller for each degree of freedom
<br>

-  **get_points():**:
	Gives the current setpoints of the controllers<br>

	Returns: list of setpoints for each controller over each degree of freedom
<br>
