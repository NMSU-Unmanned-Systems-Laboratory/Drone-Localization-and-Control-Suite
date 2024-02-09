#''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#' A program to run a 'circle traversial' expirement on two Tello drones                                                '
#' This sends control signals to fly two drones along opposite sides of a circle trajectory with both facing the center '
#''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
import time
import tf
import math

from threading import Thread

from Drone_Loc_and_Cont_Suite.drone import *

#temporary global position variable for both drones
mPos = [0, 0]

#A dictionary to easially differentiate between the two drones
drone1Name = 'tello_01'
drone2Name = 'tello_02'
nameMap = {drone1Name:1, drone2Name:2}

#callback function to receive localization data of drone and send control signals
def opti_callback(msg, drone, procedureRun):
    global mPos
    
    #break down the msg orinetation components of a quaternion
    quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    #transfor the quaternion into euler angles (roll, pitch, yaw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    #roate the yaw by 90 degrees 
    #this aligns the body to our lab's global localization system
    #WILL BE DIFFERENT FOR DIFFERENT GLOBAL FRAMES OF REFERENCE AND LOCALIZATION SOURCE/ENVIRONEMT
    yaw_rad = euler[2] - (math.pi/2)
    if (yaw_rad < -(math.pi)):
        yaw_rad += (2 * math.pi)
    
    #encapsulate data into a structure that has the x, y, and z position alond with the yaw of the dornes position.
    #The position in mPos depends on the number of the dron ethat called the callback function
    if (nameMap[drone.name] == 1):
        mPos[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    else:
        mPos[1] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]


#create two drone objects and put them in a list together
# the controllers for the drones will be initialized as the default for tello drones
myDrones = [
            Drone(name=drone1Name, type='tello', controller=None, callback=opti_callback, ip='192.168.1.141'), 
            Drone(name=drone2Name, type='tello', controller=None, callback=opti_callback, ip='192.168.10.1')
        ]

#encapsulate the created tello drones into a swarm object
Drone.create_tello_swarm()


#a function to calculate the angle between two points
#our application is to calculate angle between 
#pos and point are in format [xPos, yPos]
def angle_to_point(pos, point, hasZ = False, outDeg = True):
    #calculate differences in both x and y to get error vector
    xDiff = point[0] - pos[0]
    yDiff = point[1] - pos[1]

    #use arcTan to calcualte angle
    ang = math.atan2(yDiff, xDiff)

    #return either degrees or radians based on parameter
    if outDeg:
        return (ang * 180/math.pi)

    else:
        return(ang)


#funtion to generate a point on a circle given an angle (time), radius, and offset for y. No offset for x is avaliable currently
def genCircle(rads, r, yOff):
    #generate x coordinate of circle point
    x = r * math.cos(rads)
    #generate y coordinate of circle point
    y = r * math.sin(rads) + yOff
    
    return x, y


#thread function to wait for user input to stop and land drones.
def emergency_stop():
    input('hit enter to stop')

    Drone.procedureRun = False
    Drone.stop_and_land_drones(myDrones)

    print('emergency stopped')
    exit(1)


#main funtion
def main():
    #list of the two drone objects
    global myDrones

    #receive input to confirm takeoff of drone
    myIn = input("'t' to takeoff: ")

    #exit program if not what is expected
    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)

    #execute takeoff command for both drones
    Drone.takeoff_drones(myDrones)
    print('takeoff')

    #prompt user to start the stabilization on the point
    myIn = input("'s' to start procedure: ")

    #exit program if not what is expected
    if (myIn != 's'):
        Drone.stop_and_land_drones(myDrones)
        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    

    print('procedure start')



    #parameters for the circle and circle travversial speed
    radius = 0.75  #radius of the circle in the units of the localization system
    yOffset = 0     #y offset of the circle in the units of the localization system
    sPerRot = 15    #number of seconds for the drone to traverse the circle

    #Set the initial points of the controllers for the drones. One at the top of the cirlce, one at the bottom.
    myDrones[0].set_controller_points([radius, yOffset, 1, angle_to_point([radius,yOffset], [0, yOffset])], reset=True)
    myDrones[1].set_controller_points([-radius, yOffset, 1, angle_to_point([radius+math.pi,yOffset], [0, yOffset])], reset=True)
    #start the movement of the drones to these points
    Drone.procedureRun = True
    
    #wait for user input to start the circle movement
    input('hit enter to start rotation')

    #current radians of the circle trajectory (position along the trajectory)
    rads = 0
    
    #create and start a thread to wait for user input to stop execution while sending control signals
    thread = Thread(target=emergency_stop)
    thread.start()

    #variable for the center of the circle
    cirCent = [0, yOffset]

    #frequency to send control signals to the drone. 
    #This is the amount of time between each control signal
    freq = 0.05

    #time variable to ensure timely following of trajectory in accordance with 'sPerRot'
    lastTime = time.time()

    #control loop to follow circle trajectory for 2 full rotations or until stopped by tread funtion
    while (rads < math.pi * 4 and Drone.procedureRun):
        #timer to enforce the frequency created earlier as well as track time between loop executions
        startT = time.time()

        #calculate time since last loop execution
        diff = startT - lastTime

        #calculate the new radian value based on the time difference
        rads += diff * (math.pi*2)/sPerRot

        #calcualte x and y points along for both drones
        x1, y1 = genCircle(rads, radius, yOffset)
        x2, y2 = genCircle(rads+math.pi, radius, yOffset)

        #set the controller set points of the drones to the circle poitns calcualted
        #the yaw setpoint will be the angle between the drones current position and the circle center. calculated using angle_to_point
        myDrones[0].set_controller_points([x1, y1, 1, angle_to_point(mPos[0], cirCent)], reset=False)
        myDrones[1].set_controller_points([x2, y2, 1, angle_to_point(mPos[1], cirCent)], reset=False)

        #Using the updated setpoints, send control signals to the drones to pursue the point
        myDrones[0].send_cont(mPos[0], rotPosConts=True)
        myDrones[1].send_cont(mPos[1], rotPosConts=True)

        lastTime = startT

        #wait until the frequency time is met
        while (time.time() - startT <= freq):
            pass

        
    #if program is ending naturally
    if Drone.procedureRun:
        #set the controller points to be exactly the starting points
        myDrones[0].set_controller_points([radius, yOffset, 1, angle_to_point([radius,yOffset], [0, yOffset])], reset=True)
        myDrones[1].set_controller_points([-radius, yOffset, 1, angle_to_point([radius+math.pi,yOffset], [0, yOffset])], reset=True)

        #wait one second
        time.sleep(1)

        #stop procedure and land drones
        Drone.procedureRun = False

        print('procedure finished')
        Drone.stop_and_land_drones(myDrones)
    

    print('landing')

    #final wait before program terminates
    time.sleep(3)


#program start to create ros node and start execution of the main function
if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode', anonymous=True)
    main()
    print("Done!!")

