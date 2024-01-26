#''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#' A program to run the single point stabilization on a Tello drone '
#''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
import time
import tf
import math

import threading

from Drone_Loc_and_Cont_Suite.drone import *



#function to receive localization data of the drone from the Motion Capture System
# 'drone' and 'procedureRun' parameters are avaliable to send commands to the drone directly through this function
# in this case, only the localization is used
def callback(msg, drone, procedureRun):
    global pos

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


#thread function to wait for user input to stop the execution of the program
def thread_stop():
    input('press Enter to stop')

    Drone.procedureRun = False

    print('thread exit')


#main funtion
def main():

    #PID Parameters for the drone controller. 
    # for the x, y, z, and yaw PID controllers
    contParams = [
    	[55, 0.6, 40],
    	[55, 0.6, 40],
        [60, 0.6, 40],
    	[31.847, 0.01, 0]
    ]

    #set point for the x, y, z, and yaw positions to stabilize on
    setpoint = [0, 0.5, 1, 0]

    #Drone object construction with type 'tello'
    # also construct the controller object for the drone of type PID and 3 positional degrees of freedom
    droneCont = Controller(type='PID',numPosAxis=3, numRotAxis=1, contParams=contParams)
    myDrone = Drone(name='tello_01', type='tello', controller=droneCont, callback=callback, ip='192.168.10.1')
    
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

    #set the setpoints of the drone's controller
    myDrone.set_controller_points(setpoint)
    

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
    
    #frequency to send control signals to the drone. 
    #This is the amount of time between each control signal
    freq = 0.05

    #create and start a thread to wait for user input to stop execution while sending control signals
    t1 = threading.Thread(target=thread_stop)
    t1.start()

    #while loop over master variable
    while (Drone.procedureRun):
        #timer to enforce the frequency created earlier
        startT = time.time()
        
        #optional printing of the read position
        #print(pos)
        
        #send control signal to the drone using the read position and its controller's parameters 
        myDrone.send_cont(pos, verbose = True, rotPosVels = True)
        
        #wait until the frequency time is met
        while (time.time() - startT < freq):
            pass
    

    #master variable was turned to false by the thread function
    #stop execution and movement of drone, and land it
    myDrone.stop_movement()
    myDrone.land()
    print('procedure finished')
    print('landing...')

    #final wait before program terminates
    time.sleep(3)
   

#program start to create ros node and start execution of the main function
if __name__ == '__main__':
    print("Starting main()")
    rospy.init_node('myNode')
    main()
    print("Done!!")

