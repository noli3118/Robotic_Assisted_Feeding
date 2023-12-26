#!/usr/bin/python3

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
from sensor_msgs.msg import Joy
import transformations as T

rospy.init_node('keyboard_ikgoal_driver')

ik_goal_r_pub = rospy.Publisher('/ik_goal_r',PoseStamped,queue_size=5)
ik_goal_l_pub = rospy.Publisher('/ik_goal_l',PoseStamped,queue_size=5)
goal_pos_pub = rospy.Publisher('vive_position', Vector3Stamped)
goal_quat_pub = rospy.Publisher('vive_quaternion', QuaternionStamped)
ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)
joy_data = None


def joy_cb(data):
    global joy_data
    joy_data = data

rospy.Subscriber("joy", Joy, joy_cb)


# Old
# pos_stride = 0.015
# rot_stride = 0.055

# New
pos_stride = 0.00001
rot_stride = 0.00005




position_r = [0,0,0]
rotation_r = [1,0,0,0]

position_l = [0,0,0]
rotation_l = [1,0,0,0]

seq = 1
rate = rospy.Rate(1000)


while not rospy.is_shutdown():
    pose = PoseStamped()
    pose.pose.position.x = position_r[0]
    pose.pose.position.y = position_r[1]
    pose.pose.position.z = position_r[2]

    pose.pose.orientation.w = rotation_r[0]
    pose.pose.orientation.x = rotation_r[1]
    pose.pose.orientation.y = rotation_r[2]
    pose.pose.orientation.z = rotation_r[3]
    ik_goal_r_pub.publish(pose)

    pose = PoseStamped()
    pose.pose.position.x = position_l[0]
    pose.pose.position.y = position_l[1]
    pose.pose.position.z = position_l[2]

    pose.pose.orientation.w = rotation_l[0]
    pose.pose.orientation.x = rotation_l[1]
    pose.pose.orientation.y = rotation_l[2]
    pose.pose.orientation.z = rotation_l[3]
    ik_goal_l_pub.publish(pose)

    print("Pos R: {}, Pos L: {}".format(position_r, position_l))

    key = 'i'
    # key = readchar.readkey()
    while joy_data == None:
        print('sleep')
        rospy.sleep(1)
    # print('joy_data', joy_data)

    if abs(joy_data.axes[0]) > 0.1:
        position_r[0] -= pos_stride * joy_data.axes[0]

    if abs(joy_data.axes[1]) > 0.1:
        position_r[1] += pos_stride * joy_data.axes[1]

    if abs(joy_data.axes[4]) > 0.1:
        position_r[2] += pos_stride * joy_data.axes[4]

    if abs(joy_data.axes[6]) > 0.1:
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[0] += rot_stride * joy_data.axes[6]
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    if abs(joy_data.axes[7]) > 0.1:
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[1] += rot_stride * joy_data.axes[7]
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    if abs(joy_data.buttons[4]) > 0.1:
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[2] += rot_stride
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    if abs(joy_data.buttons[5]) > 0.1:
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[2] -= rot_stride
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])




    elif key == '3':
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[1] += rot_stride
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

    elif key == '4':
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[1] -= rot_stride
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

    elif key == '5':
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[2] += rot_stride
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '6':
        euler = list(T.euler_from_quaternion(rotation_r))
        euler[2] -= rot_stride
        rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

    elif key == 'i':
        position_l[0] += pos_stride # left
    elif key == 'm':
        position_l[0] -= pos_stride
    elif key == 'j':
        position_l[1] += pos_stride
    elif key == 'l':
        position_l[1] -= pos_stride
    elif key == 'u':
        position_l[2] += pos_stride
    elif key == 'n':
        position_l[2] -= pos_stride
    elif key == '=':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[0] += rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '-':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[0] -= rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '0':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[1] += rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '9':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[1] -= rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '8':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[2] += rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '7':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[2] -= rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == 'q':
        q = Bool()
        q.data = True
        quit_pub.publish(q)
    elif key == 'c':
        rospy.signal_shutdown()


    pose = PoseStamped()
    pose.pose.position.x = position_r[0]
    pose.pose.position.y = position_r[1]
    pose.pose.position.z = position_r[2]

    pose.pose.orientation.w = rotation_r[0]
    pose.pose.orientation.x = rotation_r[1]
    pose.pose.orientation.y = rotation_r[2]
    pose.pose.orientation.z = rotation_r[3]
    ik_goal_r_pub.publish(pose)


    pose = PoseStamped()
    pose.pose.position.x = position_l[0]
    pose.pose.position.y = position_l[1]
    pose.pose.position.z = position_l[2]

    pose.pose.orientation.w = rotation_l[0]
    pose.pose.orientation.x = rotation_l[1]
    pose.pose.orientation.y = rotation_l[2]
    pose.pose.orientation.z = rotation_l[3]
    ik_goal_l_pub.publish(pose)

    pos_goal = Vector3Stamped()
    pos_goal.vector.x = position_r[0]
    pos_goal.vector.y = position_r[1]
    pos_goal.vector.z = position_r[2]
    goal_pos_pub.publish(pos_goal)

    quat_goal = QuaternionStamped()
    quat_goal.quaternion.w = rotation_r[0]
    quat_goal.quaternion.x = rotation_r[1]
    quat_goal.quaternion.y = rotation_r[2]
    quat_goal.quaternion.z = rotation_r[3]
    goal_quat_pub.publish(quat_goal)

    ee_pose_goals = EEPoseGoals()
    pose_r = Pose()
    pose_r.position.x = position_r[0]
    pose_r.position.y = position_r[1]
    pose_r.position.z = position_r[2]

    pose_r.orientation.w = rotation_r[0]
    pose_r.orientation.x = rotation_r[1]
    pose_r.orientation.y = rotation_r[2]
    pose_r.orientation.z = rotation_r[3]

    pose_l = Pose()
    pose_l.position.x = position_l[0]
    pose_l.position.y = position_l[1]
    pose_l.position.z = position_l[2]

    pose_l.orientation.w = rotation_l[0]
    pose_l.orientation.x = rotation_l[1]
    pose_l.orientation.y = rotation_l[2]
    pose_l.orientation.z = rotation_l[3]
    ee_pose_goals.ee_poses.append(pose_r)
    ee_pose_goals.ee_poses.append(pose_l)

    ee_pose_goals.header.seq = seq
    seq += 1
    ee_pose_goals_pub.publish(ee_pose_goals)

    q = Bool()
    q.data = False
    quit_pub.publish(q)
