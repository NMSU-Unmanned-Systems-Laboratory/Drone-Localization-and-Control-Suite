#'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#' A program to run a 'wand follow expirement' on a Tello drone                                  '
#' This sends control signals to fly a drone a given distance away from where a wand is pointing '
#'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
import time
import tf
import math

from Drone_Loc_and_Cont_Suite.drone import *


#a function to calculate the angle between two points
#our application is to calculate angle between the wand and drone positions to determine what angle to point the drone towards
#pos and point are in format [xPos, yPos]
def angle_to_point(pos, point, hasZ = False, radRot = math.pi/2, outDeg = True):
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


# a function to extrapolate a point from a base point in the given direction of the position
def point_from_point(basePos, distAway = 1, zPos = None):

    # create a unit vector in the direction the drone is facing
    uVec = [math.cos(basePos[-1]), math.sin(basePos[-1])]

    #multiply the unit vector by the distance for x and y axes and add it to the original position (including or excluding z position)
    # this final value is the extrapolated point that is distAway away from the original point
    if zPos != None:
        return([basePos[0] + (uVec[0] * distAway), basePos[1] + (uVec[1] * distAway), zPos, (basePos[-1]- math.pi)* 180/math.pi])
    else:
        return([basePos[0] + (uVec[0] * distAway), basePos[1] + (uVec[1] * distAway), basePos[2], (basePos[-1]- math.pi)* 180/math.pi])


#callback function to receive localization data of drone and send control signals
def drone_callback(msg, drone, procedureRun):

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
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    

    #only execute while master variable is true
    if procedureRun:
        #set the new controller set points to that of 'newSetPoint' which is the extrapolated point from the wand end
        drone.set_controller_points(newSetPoint, reset = False)

        #Send control signals to the drone based on its controller setpoints
        drone.send_cont(pos, verbose = False, rotPosVels = True)


#callback funtion to get the loicalization data of the wand and update the global 'newSetPoint' variable
def wand_callback(msg, drone, procedureRun):
    global newSetPoint

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
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw_rad]
    
    #extrapolate a control point for the drone to follow from the wands position
    newSetPoint = point_from_point(pos, distAway=1)

    #print(f"x  : {newSetpoint[0]}")
    #print(f"y  : {newSetpoint[1]}")
    #print(f"yaw: {newSetpoint[-1] * 180/math.pi}")

        


#main funtion
def main():

    #PID Parameters for the drone controller. 
    # for the x, y, z, and yaw PID controllers
    contParams = [
        [105, 0.6, 40],
        [105, 0.6, 40],
        [100, 0.6, 10],
        [60, 0, 0]
    ]

    #Drone object construction with type 'tello'
    # also construct the controller object for the drone of type PID and 3 positional degrees of freedom
    droneCont = Controller(type='PID',numPosAxis=3, numRotAxis=1, contParams=contParams)
    myDrone = Drone(name='tello_01', type='tello', controller=droneCont, callback=drone_callback, ip='192.168.1.141', bounds=[(-1.0, 1.0),(-1.5, 1.9),(0.8, 2.0)])
    
    #Create an empty drone objet to receive the position of the wand through the wand_callback function
    Drone('wand_01', 'wand', None, wand_callback)
    
    #wait after creating drones to make sure all ros components are ready
    time.sleep(0.25)

    #receive input to confirm takeoff of drone
    myIn = input("'t' to takeoff: ")

    #exit program if not what is expected
    if (myIn != 't'):
        print(f"invalid input '{myIn}'")
        print('exiting...')
        exit(1)

    #execute takeoff command
    myDrone.takeoff()
    print('takeoff')

    #prompt user to start the stabilization on the point
    myIn = input("'s' to start procedure: ")

    #exit program if not what is expected
    if (myIn != 's'):
        myDrone.land()

        print(f"invalid input '{myIn}'")
        print('landing and exiting...')
        exit(1)
    
    

    print('procedure start')


    #master variable for the experiement to run
    Drone.procedureRun = True
    
    #wait for user to hit enter to stop the program
    input("hit enter to stop and land:")
    
    #turn master variable to false
    Drone.procedureRun = False

    #stop, land, and end execution
    myDrone.stop_movement()
    myDrone.land()

    print('procedure finished')
    print('landing')
   
    #final wait before program terminates
    time.sleep(3)


#program start to create ros node and start execution of the main function
if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode')
    main()
    print("Done!!")

