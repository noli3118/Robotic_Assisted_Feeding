#! /usr/bin/env python3
import franka_gripper.msg
import rospy
import sys
import actionlib
from sensor_msgs.msg import Joy


grip_max = 0.08
grip_min = 0.01
grip_cur = 0.08
grip_inc = 0.005


def move_gripper():
    global grip_cur
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal(width=grip_cur, speed=0.2)
    #goal.width = 0.022
    #goal.speed = 1.0
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A move result

def home_gripper():
        
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('franka_gripper/homing', franka_gripper.msg.HomingAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.HomingAction()
    #goal.width = 0.022
    #goal.speed = 1.0
    
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A move result

def joy_cb(data):
    global grip_cur
    global grip_inc
    global grip_max
    global grip_min

    a = data.buttons[0]
    b = data.buttons[1]
    if a:
        grip_cur += grip_inc
    elif b:
        grip_cur -= grip_inc
    if a or b:
        move_gripper()
    if grip_cur > grip_max: grip_cur = grip_max
    if grip_cur < grip_min: grip_cur = grip_min
    print('Grip cur', grip_cur)
    print('A:', data.buttons[0])
    print('B:', data.buttons[1])



if __name__ == '__main__':
    try:
        rospy.init_node('hiro_gripper')
        result = home_gripper()
        # result = move_gripper()
        joy_sub = rospy.Subscriber('/joy', Joy, joy_cb,queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        print("Success: ",result.success)
        print("Error message: ", result.error)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")