#!/usr/bin/env python3 
# import numpy as np
import random
import math
import copy
import rospy
from geometry_msgs.msg import Twist   #import msg data type "Twist" to be published
from std_msgs.msg import String,Int32,Float64MultiArray
from geometry_msgs.msg import Pose      #import msg data type "Pose" to be subscribed
from nav_msgs.msg import Odometry
import numpy as np                    #import numpy for trignometric function, arrays... etc
from tf.transformations import euler_from_quaternion
import sys    
                      #import sys for extracting input from termminal (input from user)
#########################################################################################################

def callback(data):
    global Current_pos	#Identify msg variable created as global variable
    global flag_initial
    msg = data ##Extract the data sent as message
    Current_pos[0] = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
    Current_pos[1]= round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    theta = round(yaw,4)
    Current_pos[2] = theta
    flag_initial = 1				#Set flag to one
	  #print("sub" + str(position[0]))
def DistancetoGoal():
    global Goal
    return DistancetoPoint(Goal)

def DistancetoPoint(point):
    global Current_pos
    dx= point[0]-Current_pos[0]
    dy= point[1]-Current_pos[1]
    dtheta = point[2]-Current_pos[2]
    return [dx,dy,dtheta]

def DidIconverge(error):
    global tolerance
    global thetaTolerance
    if error[0]<tolerance and error[1]<tolerance and error[2]<thetaTolerance:
        return True
    return False

rospy.init_node('GoalSender', anonymous=True) #Identify ROS Node
pub = rospy.Publisher('/Goal', Float64MultiArray, queue_size=1) #Identify the publisher "pub1" to publish on topic "/Tut3_Control_Input" to send message of type "Twist"
rate = rospy.Rate(5) # rate of publishing msg 10hz
sub2 = rospy.Subscriber('/odom', Odometry, callback)
Current_pos=[0,0,0]
tolerance=0.03
thetaTolerance=3*math.pi/180

data_to_send = Float64MultiArray()
i=0
#TB1 goals=[[0.32,0,90*math.pi/180],[0.32,1.9,0*math.pi/180],[1.12,1.9,0*math.pi/180],[1.12,1.9,180*math.pi/180],[-1.08,1.9,180*math.pi/180],[-1.08,1.9,-90*math.pi/180],[-1.08, 1.9, 0*math.pi/180],[0.32,1.9, -90*math.pi/180 ],[0.32,0,180*math.pi/180],[0,0,0*math.pi/180]]
goals=[[0.32,0,90*math.pi/180],[0.32,1.05,180*math.pi/180],[-1.42,1.05,-90*math.pi/180],[-1.42,1.05,0*math.pi/180],[0.32,1.05,90*math.pi/180],[0.32,1.65,0*math.pi/180],[1.12, 1.65, 0*math.pi/180], [1.12,1.65,180*math.pi/180],[0.32, 1.65, -90*math.pi/180], [0.32, 0, 180*math.pi/180], [0,0,0*math.pi/180]]
Currentgoal=goals[i]
while(1 and not rospy.is_shutdown()):

    error = np.abs(DistancetoPoint(Currentgoal))
    if i<len(goals)-1 and DidIconverge(error):
        i=i+1
        Currentgoal=goals[i]
    data_to_send.data = Currentgoal
    print("Goal,Point",data_to_send.data)
    pub.publish(data_to_send)
    #print(Current_pos)
    #print(Distance) 
    rate.sleep()