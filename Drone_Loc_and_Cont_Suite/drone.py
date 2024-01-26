import sys
import math

from pyparrot.Bebop import Bebop

from djitellopy import Tello, TelloSwarm

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

from simple_pid import PID
import numpy as np



class Controller:
    '''
        Class to define different types of controllers. 
        Controllers will calcualte their values given localization input and desired setpoints for each.

        Currently supports: PID

        Attributes:
          -list[] conts: A list of controllers for every degree of freedom
          -list[float] numPosAxis: The number of positional degrees of freedom to control, up to 3
          -list[float] numPosRot: The number of rotatoinal degrees of freedom to control, up to 3
    '''

    def  __init__(self, type:str, numPosAxis:int, numRotAxis:int, contParams:list, setPoints = None):
        """
            Creates a Controller class

            Parameters:
              -str type: the type of controller being made options:('PID')
              -int numPosAxis: The number of positional degrees of freedom to control (1 <= numPosAxis <= 3)
              -int numRotAxis: The number of rotational degrees of freedom to control (1 <= numRotAxis <= 3)
              -list[] contParams: The parameters for the controller. The structure of this will depend on parameter 'type'
              -list[floats] setPoints: list of size numPosAxis + numRotAxis. Initializes each controller's setpoint with the values of the list starting with the positional axes, then rotational

            Returns: The resulting Controller object with parameter configuration 
            
        """

        #initialize attributes of class
        self.conts = []
        self.numPosAxis = numPosAxis
        self.numRotAxis = numRotAxis
        
        #build conts object based on the type of controller
        if type.upper() == 'PID':
            self.type = 'PID'

            #parameter restriction error control
            if numPosAxis < 1:
                print("Parameter 'numAxis' for class 'Controller' must be either 1, 2, or 3", file=sys.stderr)
                sys.exit(1)

            if numRotAxis < 0:
                print("Parameter 'numAxis' for class 'Controller' must be 0 or greater", file=sys.stderr)
                sys.exit(1)

            if len(contParams) != numPosAxis + numRotAxis:
                print("Parameter 'contParams' for Controller type 'PID' requires parameters for each axis", file=sys.stderr)
                sys.exit(1)

            if len(contParams[0]) != 3:
                print("Parameter 'contParams' for Controller type 'PID' requires 3 values for each axis (Proportunal, Integral, Derivative)", file=sys.stderr)
                sys.exit(1)

            #create a PID controller for each set of parameters (proportunal, integral, derivative constants) received in contParams
            for params in contParams:
                self.conts.append(PID(params[0], params[1], params[2]))
            
            #set the controller points if initializing parameter is used
            if setPoints != None:
                self.set_points(setPoints)


    #class function to 
    def rotation_goal_standardize(self, goal, out = True):
        """
            Transforms any amount of degrees to the degree or radian range of -180 to 180

            Parameters:
              -float goal: The angle in degrees to convert to the range -180 to 180
              -bool out: an int signifying whether to return RAD or Degrees. (default: True)
                  True represents a return value in Radians
                  False represents a return value in Degrees 

            Returns: returns a float in the degree or radian range of -180 to 180
        """

        #remove extrenuous rotations in goal
        goal %= 360
        
        #translate to range -180 < h < 180
        if (goal < -180):
            goal += 360
        elif (goal > 180):
            goal -= 360
            
        #return radians or degrees
        if (out):
            return goal * math.pi / 180
        else:
            #return in Degrees
            return goal


    def set_points(self, setPoints:list, reset = True):
        """
            Sets the set-points of all internal degree of freedom controllers to the points given

            Parameters:
              -list setPoints: List of new set-points for each degree of freedom
              -bool reset: A boolean to represent whether to reset the cache of the controllers with the new set-points
        """

        #set points according to the type of controller
        if self.type == 'PID':
            if len(setPoints) != len(self.conts):
                print("Number of points in parameter 'setPoints' do not match number of axis", file=sys.stderr)
                sys.exit(1)

            #Covert rotational set-point to the working range of -180 to 180 degrees
            for i in range(self.numPosAxis, self.numPosAxis + self.numRotAxis):
                setPoints[i] = self.rotation_goal_standardize(setPoints[i])
            
            #Change the setpoitn of every PID controller
            for i in range(len(setPoints)):
                #reset the controller if reset is True
                if reset:
                    self.conts[i].reset()
                self.conts[i].setpoint = setPoints[i]


    def get_optimal_rot_pos(self, rot_curr, goal):
        """
            Given a current rad posiiton and a goal rad position in the range -pi to pi, calculate whether it is shorter to travel to the goal via positive or negative rotation

            Parameters:
              -float rot_curr: current rotational position in radians
              -float rot_curr: rotational goal in radians

            Returns: float representing the new current rotational position that corrosponds to the shortest path to reach the goal

            ex1: given a position of 175 degrees and a goal of -175 degrees, the function would return -185 as an altered current rotational position
            ex2: given a position of 23 degrees and a goal of 54 degrees, the function would return 23 as the current rotational position
        """
        
        #calculate the difference between the goal and the current position
        diff = goal - rot_curr
        
        #if difference is less than PI (or 180 degress), then the optimal path is the current value
        if (abs(diff) <= math.pi):
            return rot_curr
        #else, the reverse path is more optimal and we need to feed the PID a value that is shifted by 360 degrees accordingly
        # (ie the degree > |180| that is equivalent and 'closer' to the goal)
        elif (diff <= 0):
            return rot_curr - (2*math.pi)
        else:
            return rot_curr + (2*math.pi)

    
    def get_out(self, pos:list):
        """
            Gives outs of the controller for each axis given current position

            Parameters:
              -list pos: current position of controlled device for each degree of freedom 

            Returns: list output of each controller for each degree of freedom
        """

        #get outputs depending on the type of controller
        if self.type == 'PID':
            #parameter error checking
            if len(pos) != len(self.conts):
                    print("Number of points in parameter vector 'pos' do not match number of axis", file=sys.stderr)
                    sys.exit(1)
            

            #empty structure to fill and return
            outs = [0] * len(pos)

            #iterate through each controller and calculate an output given the current position
            #starting with positional controllers
            for i in range(self.numPosAxis):
                outs[i] = self.conts[i](pos[i])
            #then rotational controlelrs (utilizing the get_optimal_rot_pos function)
            for i in range(self.numPosAxis, self.numPosAxis + self.numRotAxis):
                outs[i] = self.conts[i](self.get_optimal_rot_pos(pos[i], self.conts[i].setpoint))

            return outs
    

    def get_points(self):
        """
            Gives the current setpoints of the controllers

            Returns: list of setpoints for each controller over each degree of freedom
        """
        out = None

        #put all the current controller setpoints into the 'out' variable depending on the type of controller 
        if self.type == 'PID':
            out = [cont.setpoint for cont in self.conts]

        return out
                
        
        




class Drone:
    '''
        Class to abstract the control and localization of quadrotorcraft devices.

        Currently supports: PID

        Attributes:
          -bool procedureRun: A classwide variable to control the start and ending of experiements
          -str name: The unique name for the drone
          -int type: The type of drone for communication and control purposes
          -Controller controller: The Controller object to be used when sending control signals
          -function callbackFunc: The callback function that is used when localization data is received from the motion capture system
          -list[] bounds: If working in a restrained area, the box to contain all control signals within
    '''

    #A classwide variable to control the start and ending of experiements
    procedureRun = False
    #a dictionary to improve the performance when checking for the type of drone
    typeMap = {'bebop1':1, 'bebop2':1, None:None, 'pyBebop':2, 'tello':3, 'wand':4}
    #object to send commands to multiple Tellos at once
    telloSwarm = None
    #list of Tello type drones as they are created
    myTellos = []


    def  __init__(self, name:str, type:str, controller:Controller == None, callback, bounds = None, ip = None):
        """
            Creates a Drone class

            Parameters:
              -str name: The unique name for the drone. Must be the same name used for the ROS topic localization to receive localization data from the motion capture system.
              -str type: The type of drone for communication and control purposes (options: 'bebop1', 'bebop2', 'pyBebop', 'tello', 'wand', None)
              -Controller controller: The Controller object to be used when sending control signals
              -function callbackFunc: The callback function that is used when localization data is received from the motion capture system (must have 3 parameters)
              -list[()] bounds: If working in a restrained area, the box to contain all control signals within (default = None)
              -str ip: If communicating directly with drone, the ip of the drone. Needed for types 'pyBebop' and 'tello'. (default = None)

            Returns: The resulting Drone object with parameter configuration 
            
        """

        #Put all parameters into Class variables
        self.name = name
        self.controller = controller
        self.callbackFunc = callback
        self.rosTwist = Twist()
        self.bounds = bounds

        #if the type is supported, assign the type variable its associated integer using the type map
        #This avoids the need to compare strings during execution, when checking for drone type
        if type in self.typeMap:
            self.type = self.typeMap[type]
        else:
            print(f"type '{type}' not supported")
            sys.exit(1)

        #make sure the dimension of bounds is equal to the number of positional axes.
        if bounds != None and (len(bounds) != self.controller.numPosAxis or False in [len(a) == 2 for a in bounds]):
            print(f"Parameter 'bounds' must contain a list of 2D tuples where each tuple is the upper and lower bounds of each positional axis!", file=sys.stderr)
            sys.exit(1)

        #Create a subscriber for the optitrack system
        #This will call the callback function every time positional data is received for the object
        #The name of the function will be used in the ROS topic
        if self.callback != None:
            rospy.Subscriber(f'/mocap_node/{self.name}/Odom', Odometry, self.callback, queue_size=1)


        #If controller was empty, enter default controllers based on the type given
        if self.controller == None:
            if (self.type == self.typeMap['pyBebop'] or self.type == self.typeMap['bebop1'] or self.type == self.typeMap['bebop2']):
                self.controller = Controller('PID', 3, 1, 
                                                    [[0.35, 0.006, 0.4],
                                                    [0.35, 0.006, 0.4],
                                                    [0.6, 0.003, 0.2],
                                                    [1, 0, 0]])
                
            elif (self.type == self.typeMap['tello']):
                self.controller = Controller('PID', 3, 1, 
                                                    [[105, 0.6, 40],
                                                    [105, 0.6, 40],
                                                    [100, 0.6, 10],
                                                    [60, 0, 0]])


        #create publishers based on drone type
        # This is how control signals are sent to some drone types
        if (self.type == self.typeMap['bebop1'] or self.type == self.typeMap['bebop2']):
            #vel publisher
            self.velPub = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=1)
            #landing publisher
            self.landPub = rospy.Publisher(f'/{self.name}/land', Empty, queue_size=5)
            #takeoff Publisher
            self.takeoffPub = rospy.Publisher(f'/{self.name}/takeoff', Empty, queue_size=5)
                                            

        #If the drone is of type pyBebop, create a pyparrot bebop object
        if (self.type == self.typeMap['pyBebop']):
            
            self.ip = ip

            if (ip == None):
                print("'ip' parameter required for pyBebop type", file=sys.stderr)
                sys.exit(1)

            print("connecting")

            self.droneObj = Bebop(ip_address=self.ip)

            print(self.droneObj.connect(10))

            self.droneObj.flat_trim(0)

        #If the drone is of type tello, create a djitello tello object
        elif (self.type == self.typeMap['tello']):
            self.ip = ip
            self.droneObj = Tello(host=self.ip)
            self.droneObj.connect()

            Drone.myTellos.append(self.droneObj)


        #if the type is wand, there is no control signals sent
        elif (self.type == self.typeMap['wand']):
            pass


        #else the type is unsupported
        else:
            print(f"Unknown type '{type}' for class Drone", file=sys.stderr)
            sys.exit(1)



    def set_controller_points(self, setPoints:list, reset = True):
        """
            Sets the setpoints of the drones controllers. Taking into account the bounds of the Drone object

            Parameters:
              -list setPoints: List of new set-points for each degree of freedom
              -bool reset: A boolean to represent whether to reset the cache of the controllers with the new set-points
        """

        #If there are bounds set
        if self.bounds != None:
            #truncate all of the setpoints within the designated bounds
            for i in range(self.controller.numPosAxis):
                setPoints[i] = max(self.bounds[i][0], min(self.bounds[i][1], setPoints[i]))

        #call set_points function of the drone's Controller object
        self.controller.set_points(setPoints, reset = reset)






    def rotate_conts(self, v, rotation):
        """
            Takes positional control and rotates by a certain amount

            Parameters:
              -list[float] v: Positional control signal vector. One value for each positional degree of freedom
              -float rotation: The rotation amount in radians. Rotation is along the z axis.
        """

        #create a rotation matrix based on the given angle
        rot_mat = [
            [math.cos(rotation), -math.sin(rotation), 0], 
            [math.sin(rotation), math.cos(rotation), 0],
            [0, 0, 1]
            ]
        
        #Multiply the matrix by the control vector and return the new rotated control vector
        return np.dot(rot_mat, v)



    def send_cont(self, pos:list, rotPosConts:bool = True, euler:bool = True, rotAxis:str = 'yaw', verbose = False, raw_conts = None):
        """
            Sends global control signals in accordance with a given position, and the controller/setpoint. Account for necessary rotations if desired

            Parameters:
              -list[float] pos: Positional control signal vector. One value for each positional degree of freedom
              -bool rotPosConts: Rotate the given control vector to the global frame (Default = True)
              -bool euler: If the orientation of the position is in Euler radians. (Default = True)
              -str rotAxis: Axis of rotation (default = 'yaw')
              -bool verbose: if True, will print the control signals sent to the drone
              -list[float] raw_conts: If not None, will send the raw control values of the parameter directly to the drone. Must be of size numPosAxis + numRotAxis
        """


        #conts and temp order is [x, y, z, yaw, pitch, roll]
        
        temp = [0] * 4
        
        #If raw_counts exist, try to send those controls directly to the drone
        if raw_conts != None:
            #check for dimensionality of the parameter raw_conts
            if (len(raw_conts) != (self.controller.numPosAxis + self.controller.numRotAxis)):
                print("'raw_conts' parameter formatted incorrectly. Must be of size (numPosAxis + numRotAxis)", file=sys.stderr)
                sys.exit(1)

            #assign to conts variable
            conts = raw_conts

        #if not raw_conts, get the control values from the controllers
        else:
            conts = self.controller.get_out(pos)

        #if rotating the control signals to the global frame
        if rotPosConts:
            if (self.controller.numRotAxis == 0):
                print("Cannot rotate velocities if there is no rotational axis defined in Controller", file=sys.stderr)
                sys.exit(1)

            # if working with euler radians
            if euler:
                if rotAxis == 'yaw':

                    #depending on if rotations are in a 2 or 3 dimensional space, call the rotate_conts function
                    if self.controller.numPosAxis == 3:
                        v_rot = self.rotate_conts([[conts[0]], [conts[1]], [conts[2]]], -pos[3])
                    elif self.controller.numPosAxis == 2:
                        v_rot = self.rotate_conts([[conts[0]], [conts[1]], [0]], -pos[2])
                    else:
                        print("Can only rotate velocity vecotrs of 3 and 2", file=sys.stderr)
                        sys.exit(1)
                else:
                    print(f"No support for rotations along axis {rotAxis}", file=sys.stderr)
                    sys.exit(1)

            else:
                print("Currently do not support non-euler values", file=sys.stderr)
                sys.exit(1)

            #place the resulting rotated control values into the temp variable
            temp[0] = v_rot[0]
            temp[1] = v_rot[1]
            temp[2] = v_rot[2]

            

        #else apply the controller outputs directly to the temp variable
        else:
            if self.controller.numPosAxis == 3:
                temp[0] = conts[0]
                temp[1] = conts[1]
                temp[2] = conts[2]

            elif self.controller.numPosAxis == 2:
                temp[0] = conts[0]
                temp[1] = conts[1]
            else:
                temp[0] = conts[0]


        #if controlling yaw add it to the temp varaible
        if self.controller.numRotAxis > 0:
            temp[3] = conts[self.controller.numPosAxis]



        #apply the control signals based on the type of drone
        #bebop drones get a ROS topic published for control
        if (self.type == self.typeMap['bebop1']):
            self.rosTwist.linear.x = temp[0]
            self.rosTwist.linear.y = temp[1]
            self.rosTwist.linear.z = temp[2]
            self.rosTwist.angular.z = temp[3]
            self.velPub.publish(self.rosTwist)

        #pyparrot objects have a function to send control
        elif (self.type == self.typeMap['pyBebop']):
            #self.droneObj.fly_direct(-self.rosTwist.linear.y*100, self.rosTwist.linear.x*100, self.rosTwist.angular.z*100, self.rosTwist.linear.z*100, 0.01)
            self.droneObj.fly_direct(temp[0]*-100, temp[1]*100, temp[3]*100, temp[2]*100, 0.01)
        
        #tello drones also have a function to send control
        elif (self.type == self.typeMap['tello']):
            self.droneObj.send_rc_control(int(round(temp[1][0])*-1), int(round(temp[0][0])), int(round(temp[2][0])), int(round(temp[3])*-1))


        #print the control signals if asked to
        if (verbose):
            print("---------------------")
            print(f"x cont:::::::{temp[0]}")
            print(f"y cont:::::::{temp[1]}")
            print(f"z cont:::::::{temp[2]}")
            
            if (self.controller.numRotAxis > 0):
                print(f"yaw cont:::::{temp[3]}")

            
        
    def stop_movement(self):
        """
            Sends a command of zero control to the Drone
        """

        #send the zero command based on drone type 
        if (self.type == self.typeMap['bebop1']):
            stopTwist = Twist()

            stopTwist.linear.x = 0
            stopTwist.linear.y = 0
            stopTwist.linear.z = 0
            stopTwist.angular.z = 0

            self.velPub.publish(self.rosTwist)

        elif (self.type == self.typeMap['pyBebop']):
            self.droneObj.fly_direct(0, 0, 0, 0, 1)

        elif (self.type == self.typeMap['tello']):
            self.droneObj.send_rc_control(0, 0, 0, 0)



    def callback(self, msg):
        """
            Calls given callback function which requires three parameters:

            Required Parameters:
              -msg: The motiona capture position msg
              -drone: The drone object that the msg was called on
              -procedureRun: A classwide variable to control the start and ending of experiements
              
        """
        self.callbackFunc(msg, drone=self, procedureRun=Drone.procedureRun)


    def land(self, safe=True):
        """
            Executes command to land the drone

            Parameters:
              -bool safe: For use with pyparrot drones. when True executes safe_land
              
        """

        #Execute land command based on drone type
        if (self.type == self.typeMap['bebop1']):
            self.landPub.publish(Empty())
        elif (self.type == self.typeMap['pyBebop']):
            self.droneObj.safe_land(10)
        elif (self.type == self.typeMap['tello']):
            self.droneObj.land()


    
    def disconnect(self):
        """
            For use with pyparrot and Tello drones. Ends the communication and disconnects the drone.
        """

        #Execute disconnect command based on type
        if (self.type == self.typeMap['pyBebop']):
            self.droneObj.disconnect()
        elif (self.type == self.typeMap['tello']):
            self.droneObj.end()


    def takeoff(self, safe=True):
        """
            Executes command to takeoff the drone

            Parameters:
              -bool safe: For use with pyparrot drones. when True executes safe_land
              
        """

        #Execute land command based on drone type
        if (self.type == self.typeMap['bebop1']):
            self.takeoff.publish(Empty())
        elif (self.type == self.typeMap['pyBebop']):
            self.droneObj.safe_takeoff(8)
        elif (self.type == self.typeMap['tello']):
            self.droneObj.takeoff()


    #function to return if the drone is at the desired position.
    def at_setpoint(self, pos, admittedErrs):
        """
            Executes command to takeoff the drone

            Parameters:
              -list[float] pos: position along every degree of freedom
              -list[float] admittedErrs: list of absolute error allowances for each degree of freedom to assume the drone has reached its setpoint 

            Returns: bool True if the position provided is within admitted Errs distance of every degree of freedom. Otherwise False.

        """

        #get durrent setpoints for the dron's controller
        setPoints = self.controller.get_points()
        
        #makesure input pos has the same size as the number of setpoints
        if len(pos) != len(setPoints):
            print("Number of points in parameter vector 'pos' do not match number of axis", file=sys.stderr)
            sys.exit(1)

        #iterate through each point and check if the error between the current position and the setpoint
        for i, point in enumerate(setPoints):
            #return False if its greater than the admittedErr for that axis
            if abs(point - pos[i]) > admittedErrs[i]:
                return False
        
        #Else all points are within admitted range
        #Return true that the current posiiton is close enough to the setpoint
        return True


    def takeoff_drones(drones:list):
        """
            Executes takeoff command on every drone object in list parameter

            Parameters:
              -list drones: list of Drone objects to execute the takeoff command on
              
        """

        #Iterate through the list calling takeoff on all drone objects
        for d in drones:
            if d.type != Drone.typeMap['tello']:
                d.takeoff()

        #For multiple tello drones, use the telloSwarm object
        if Drone.telloSwarm != None:
            Drone.telloSwarm.takeoff()



    def stop_and_land_drones(drones:list):
        """
            Executes the stop_movement and land commands on every drone object in list parameter

            Parameters:
              -list drones: list of Drone objects to execute the commands on
              
        """
    
        #Iterate through the list calling stop_movemnt on all drone objects
        for d in drones:
            d.stop_movement()

        #Iterate through the list calling land on all drone objects
        for d in drones:
            if d.type != Drone.typeMap['tello']:
                d.land()

        #For multiple tello drones, use the telloSwarm object 
        if Drone.telloSwarm != None:
            Drone.telloSwarm.land()


    def create_tello_swarm():
        """
            Call this after creating all your Drone objects of type 'tello'. This encapsulates the tello drone objects into a tello swarm to execute commands to the tello drones simulteneously
        """

        #take all tello drone objects and encapsualte in tello swarm class variable
        Drone.telloSwarm = TelloSwarm(Drone.myTellos)




    
