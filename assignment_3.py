#!/usr/bin/env python3

#---------------Import Libraries and msgs---------------:

import rospy
import time
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from collections import Counter
from control_lib import UR_Controller


#---------------Initialise---------------:

print("Please Wait While System Starts Up...")
rospy.init_node("example_assignment", anonymous=False)
ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
ur_con = UR_Controller()
time.sleep(2)
print("System Started")

#---------------Main Code---------------:

# Home robot
def home_robot():
    home_waypoint = [-1.58825, -1.71042, -2.19911, -0.802851, 1.58825, -0.03106686]
    command = ur_con.generate_move_j(home_waypoint)
    ur_script.publish(command)
    time.sleep(5) # TO DO replace this with check

#set the robot in its home point
home_robot()

#Get the initial errors
my_pos = ur_con.get_pose()
errors = ur_con.check_errors(my_pos)
print("Cartesian error:", (errors.ee_trans_error),"m") #Directional error in meters
print("Orientational error:", errors.ee_rot_error, "degrees") #Rotational error in degrees


#defines which axis to move and value by suming up the command
def move_axis(axis, value):
    if axis == 'x':
        my_pos = ur_con.get_pose()
        my_pos.position.x += value
        command = ur_con.generate_move_l(my_pos)
        ur_script.publish(command)
        time.sleep(4)
    elif axis == 'y':
        my_pos = ur_con.get_pose()
        my_pos.position.y += value
        command = ur_con.generate_move_l(my_pos)
        ur_script.publish(command)
        time.sleep(4)
    elif axis == 'z':
        my_pos = ur_con.get_pose()
        my_pos.position.z += value
        command = ur_con.generate_move_l(my_pos)
        ur_script.publish(command)
        time.sleep(4)

#sums up the command to rotate the robot arm
def rotate_tool(r, p, y):
    command = ur_con.rotate_tool(r, p, y) 
    ur_script.publish(command)
    time.sleep(4)

#this defines which rotational axis to choose from
def rot_robot(index, value):
    if index == 0:
        rotate_tool(-value, 0, 0)
    elif index == 1:
        rotate_tool(0, value, 0)
    elif index == 2:
        rotate_tool(0, 0, -value)

#translational value to provide move_axis
axis_value = 0

#rotational value to provide rotate_tool
rot_value = 0.1 #1.5708 = 90 deg

#this is used to define the initial translational value
if errors.ee_trans_error <= 2.5:
    axis_value = 0.1
else:
    axis_value = 0.25

#list with axis to loop through
axis = ('x', 'y', 'z')

#dictionary used to store the amount that each axis did by moving a certain amount
axis_gaps = {}

#go through each axis to check which one creates the biggest difference in the error
for i, v in enumerate(axis):
    home_robot()

    #check translational error
    old_error = errors.ee_trans_error

    #move robot
    move_axis(v, axis_value) 

    #check new translational error
    my_pos = ur_con.get_pose()
    errors = ur_con.check_errors(my_pos)
    new_error = errors.ee_trans_error

    #calculate gap in error: old error - new error
    error_gap = old_error - new_error

    #add that gap and axis to dict axis_gaps
    axis_gaps[v] = error_gap

#sorting out the dictionary axis_gaps by the value by descending order
sorted_axis_gaps = sorted(axis_gaps.items(), key=lambda x:x[1], reverse=True)
#converting the list back to a dictionary
axis_gaps_converted = dict(sorted_axis_gaps)

#---------TRANSLATIONAL TRANSFORM------------#

trans_target = False
trans_index = 0

# while ball is not caught keep looping to get closer to catch the ball
while trans_target == False:
    #reset index back to 0 so it doesn't get out of bonds
    if trans_index == 3:
        trans_index = 0

    #start the error difference as 1 to prevent errors in the while loop
    error_difference = 1

    #this while loop keeps the robot moving in the same axis until it stops 
    #getting closer to the trajectory and switches to another axis
    while error_difference > 0:
        #check error 
        my_pos = ur_con.get_pose()
        errors = ur_con.check_errors(my_pos)
        old_error2 = errors.ee_trans_error

        #move robot in the axis with the biggest gap
        move_axis(list(axis_gaps_converted.keys())[trans_index], axis_value)        

        #check new error -- new error = error
        my_pos = ur_con.get_pose()
        errors = ur_con.check_errors(my_pos)
        new_error2 = errors.ee_trans_error
        print(f'This is the new translational error: {new_error2} m')

        #making the rotational values smaller as the robot gets closer
        if errors.ee_trans_error <= 0.06:
            axis_value = 0.001
           # original axis_value = 0.001
        elif errors.ee_trans_error <= 0.1:
            axis_value = 0.005
           #original axis_value = 0.005
        elif errors.ee_trans_error <= 0.2:
            axis_value = 0.025
            #original axis_value = 0.025
        elif errors.ee_trans_error <= 0.5:
            axis_value = 0.05
            #orirginal axis_value = 0.05

        #calculate gap in error -- old error - new error
        error_difference = old_error2 - new_error2

        #if the move that was applied didn't get the robot any closer, undo the move
        if error_difference <= 0:
            move_axis(list(axis_gaps_converted.keys())[trans_index], -axis_value)
        
        #needed to break out of the while loop
        if new_error2 < 0.05:
            break

    #need to update the index
    trans_index += 1

    #exit the loop
    if new_error2 < 0.05:
        break

print('Robot is within the cartesian threshold!')

#---------ROTATIONAL TRANSFORM------------#
my_pos = ur_con.get_pose()
errors = ur_con.check_errors(my_pos)
rot_error = errors.ee_rot_error

#Variables
rot_target = False
rot_index = 0

while rot_target == False:

    #reset rot_index to loop again
    if rot_index == 3:
        rot_index = 0

    rot_error_difference = 1

    while rot_error_difference > 0:
        #check error -- old error = error
        my_pos = ur_con.get_pose()
        errors = ur_con.check_errors(my_pos)
        old_rot_error = errors.ee_rot_error

        #rotate robot
        rot_robot(rot_index, rot_value)
        time.sleep(5)

        #check new error -- new error = error
        my_pos = ur_con.get_pose()
        errors = ur_con.check_errors(my_pos)
        new_rot_error = errors.ee_rot_error
        print(f'This is the new rotational error: {new_rot_error} degrees')

        #making the rotational values smaller as the robot gets closer
        if errors.ee_rot_error <= 7:
            rot_value = 0.008
        elif errors.ee_rot_error <= 10:
            rot_value = 0.0175
        elif errors.ee_rot_error <= 25:
            rot_value = 0.05

        #calculate gap in error -- old error - new error
        rot_error_difference = old_rot_error - new_rot_error
        if rot_error_difference <= 0:
            rot_robot(rot_index, -rot_value)

        #exit the loop
        if new_rot_error < 5:
            break

    #updating the index
    rot_index += 1

    #exit the loop
    if new_rot_error < 5:
        break

print('Robot is within both thresholds!')
print("Ball caught:",errors.ball_caught)