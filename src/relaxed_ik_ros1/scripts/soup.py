#!/usr/bin/python3

import rospy
import rospkg
import actionlib
from relaxed_ik_ros1.msg import EEVelGoals
from robot import Robot
from sensor_msgs.msg import Joy
import franka_gripper.msg
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
import hiro_grasp
from klampt.math import so3
import numpy as np
import fcl
import copy
from sklearn.preprocessing import normalize
from visualization_msgs.msg import MarkerArray
from pose_combinations import compute_grasp_list
import datetime


path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'
class GraspLoop:
    def __init__(self, flag, grasp_pose, grasp_apprach, 
                 # TODO: fix the home and drop quat values, the robot gets twisted
                 home_pose=[0.30871, 0.000905, 0.48742, 0.9994651, -0.00187451, 0.0307489, -0.01097748], 
                 drop_pose = [0.23127, -0.5581, 0.31198, 0.9994651, -0.00187451, 0.0307489, -0.01097748], 
                 cone_radius=0.25, cone_height=0.25,):
        self.grasp_dict = {
            "x_g" : grasp_pose,
            "x_a" : grasp_apprach,
            # "x_h" : home_pose,
            "x_d" : drop_pose,
            "x_c" : None,
            "c_r" : cone_radius,
            "c_h" : cone_height,
            "x_goal": None,
            "grasp": False
        }

        # x, y, z in cm
        # euler angle in degrees

        start_ee_position_experiment = [3.424549298862583, -58.49294620503255, 37.437693122514504, -178.8479662, -1.1794894, 46.7802049]
        start_ee_position_demo = [3.424549298862583, -58.49294620503255, 20, -178.8479662, -1.1794894, 46.7802049]
        
        end_goal_ee_position1 = [45, 25.10425547246271, 23.792718303142827, -173.8439543, -9.7114613, -45.186522]
        end_goal_ee_position2 = [45, -30, 27.792718303142827, -180, 0, -300]
        end_goal_ee_position3 = [45, -30, 27.792718303142827, -180, 0, -200]

        end_goal_ee_position_demo = [45, 25.10425547246271, 23.792718303142827, -173.8439543, -9.7114613, -100.186522]


        start_ee_position = start_ee_position_demo
        end_goal_ee_position = end_goal_ee_position1

        person_occupied = False
        if person_occupied:
            end_goal_ee_position = [end_goal_ee_position[0] - 20, end_goal_ee_position[1] - 20, 20, end_goal_ee_position[3], end_goal_ee_position[4], end_goal_ee_position[5]]



        step_constant = 15
        # compute grasp list returns in meters and quaternions
        self.grasp_list =  compute_grasp_list(start_position=start_ee_position, end_position=end_goal_ee_position, step_constant=step_constant)
        print("Grasp list: ")
        print(self.grasp_list)
        print("----")
        self.total_time = []

        self.cur_list_idx = 0
        self.__hiro_g = hiro_grasp.hiro_grasp()
        self.__pose_order_list = self.set_pose_order_list(flag)
        self.__cur_idx = 0 
        self.already_collided = False
        self.x_history_list = []
        self.y_history_list = []
        self.z_history_list = []
        self.max_history_len = 50

    def set_x_c(self, x_c):
        self.grasp_dict["x_c"] = x_c
    
    def set_x_g(self, x_g):
        self.grasp_dict["x_g"] = x_g
    
    def set_x_a(self, x_a):
        self.grasp_dict["x_a"] = x_a

    def get_curr_error(self, x_ee):
        return self.xyz_diff(x_ee, self.grasp_dict["x_goal"])
    
    def xyz_diff(self, start_pose, end_pose):
        goal = self.grasp_dict["x_goal"]
        difference = [0.0, 0.0, 0.0]
        difference[0] = goal[0] - start_pose[0]
        difference[1] = goal[1] - start_pose[1]
        difference[2] = goal[2] - start_pose[2]
        return difference
    
    def add_to_xyz_history(self, x_ee):
        self.x_history_list.append(x_ee[0])
        self.y_history_list.append(x_ee[1])
        self.z_history_list.append(x_ee[2])
        if len(self.x_history_list) > self.max_history_len:
            self.x_history_list.pop(0)
            self.y_history_list.pop(0)
            self.z_history_list.pop(0)

    def get_franka_xyz_history(self):
        return self.x_history_list, self.y_history_list, self.z_history_list

    def set_pose_order_list(self, flag):
        self.flag = flag
        if self.flag == "linear":
            self.grasp_dict["x_goal"] = self.grasp_dict["x_g"]
            return ["x_g", "grasp", "x_h", "x_d", "x_h"]
            
        elif self.flag == "l-shaped":
            self.grasp_dict["x_goal"] = self.grasp_dict["x_a"]
            return ["x_a", "x_g", "grasp", "x_h", "x_d", "x_h"]
        
        elif self.flag == "cone":
            return ["x_c", "x_g", "grasp", "x_h", "x_d", "x_h"]
        
        elif self.flag == "list":
            self.grasp_dict["x_goal"] = self.grasp_list[self.cur_list_idx]
            return None
        
    def update_grasp_goal(self):
        self.grasp_dict["x_goal"] = self.grasp_dict[self.__pose_order_list[self.__cur_idx]]

    def set_grasp_width(self, width):
        self.__hiro_g.set_grasp_width(width)
    
    def grasp(self):
        self.__hiro_g.grasp()

    def get_error_xyz(self, x_ee):
        return self.xyz_diff(x_ee, self.grasp_dict["x_goal"])
    
    def check_cone_done(self):
        if self.flag == "cone" and self.__cur_idx > 2:
            return True
        return False


    def check_next_state(self, error):
        error_bound = 0.02
        error_sum = sum([abs(elem) for elem in error])
        ret = False    
        
        # print("error sum: ", error_sum)
        if error_sum < error_bound:
            if not self.__pose_order_list:
                self.cur_list_idx += 1 
                # self.cur_list_idx = self.cur_list_idx % len(self.grasp_list)
                if self.cur_list_idx >= len(self.grasp_list):
                    self.grasp_dict["x_goal"] = self.grasp_list[-1]

                    start_time = self.total_time[0]
                    end_time = self.total_time[-1]
                    time_difference = end_time - start_time
                    
                    print("Total time from start to goal: ", time_difference)

                    # print("Time taken is for each mp: ", self.total_time)
                    return True 
                else:
                    now = datetime.datetime.now()
                    self.total_time.append(now)

                    print("self.cur: ", self.cur_list_idx)
                    if self.cur_list_idx == 3:
                        rospy.sleep(10.0)

                    self.grasp_dict["x_goal"] = self.grasp_list[self.cur_list_idx]
                    return ret
            if self.__pose_order_list[self.__cur_idx] == "x_d":
                self.__hiro_g.open()
                rospy.sleep(1.0)
            ret = self.inc_state()
        if not self.__pose_order_list:
            return ret
        elif self.__pose_order_list[self.__cur_idx] == "grasp":
            ret = self.inc_state()
            self.__hiro_g.set_grasp_width(0.055)
            self.__hiro_g.grasp()
            rospy.sleep(1.0)
        return ret
    
    def __wait_for_new_grasp_pose(self):
        while True:
            self.__check_new_grasp()
            rospy.sleep(1)

    def inc_state(self):
        self.__cur_idx += 1 
        self.__cur_idx = self.__cur_idx % (len(self.__pose_order_list))
        self.grasp_dict["x_goal"] = self.grasp_dict[self.__pose_order_list[self.__cur_idx]]
        if self.__cur_idx == 0:
            self.already_collided = False
            return True
        return False

class XboxInput:
    def __init__(self, flag):
        self.client = actionlib.SimpleActionClient('franka_gripper/move', franka_gripper.msg.MoveAction)
        self.flag = flag

        default_setting_file_path = path_to_src + '/configs/settings.yaml'

        setting_file_path = rospy.get_param('setting_file_path')
        if setting_file_path == '':
            setting_file_path = default_setting_file_path

        self.robot = Robot(setting_file_path)

        self.grasped = False
        self.final_location = False
        self.grasp_loop = False
        self.made_loop = False

        self.ee_vel_goals_pub = rospy.Publisher('relaxed_ik/ee_vel_goals', EEVelGoals, queue_size=1)
        self.hiro_ee_vel_goals_pub = rospy.Publisher('relaxed_ik/hiro_ee_vel_goals', Float64MultiArray, queue_size=1)

        self.p_x = 0.04 # 0.015
        self.p_y = 0.02 # 0.01
        self.p_z = 0.022 # 0.022
        
        self.p_r = 0.001

        self.rot_error = [0.0, 0.0, 0.0]

        self.z_offset = 0.0
        self.y_offset = 0.0
        self.x_offset = 0.0
        self.seq = 1
        
        self.linear = [0,0,0]
        self.angular = [0,0,0]
        self.joy_data = None
        self.grasp_pose = [0,0,0,0,0,0,0]
        self.x_a = [0,0,0,0,0,0,0]
        self.start_grasp = False
        self.prev_fr_euler = [0, 0, 0]
        self.grasp_midpoint = [0,0,0,0,0,0,0]

        self.grip_cur = 0.08
        self.grip_inc = 0.01
        self.grip_max = 0.08
        self.grip_min = 0.01

        self.fr_position = [0.0, 0.0, 0.0]
        self.fr_rotation_matrix = [[0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0]]
        self.fr_state = False
        self.error_state = [0.0, 0.0, 0.0]
        self.cur_list_idx = 0

        self.fr_is_neg = False
        self.fr_set = False
        self.got_prev_sign = False
        self.og_set = False
        self.in_collision = False
        self.og_x_a = [1, 1, 1]
        self.cone_radius = 0.25
        self.cone_height = 0.25
        self.msg_obj_to_line = 0.0
        self.msg_cone_start = 0.0
        self.nearest_cone_points = [None, None]
        self.x_c = [0,0,0,0,0,0,0]
        # self.cone_pub = rospy.Publisher('cone_viz', MarkerArray, queue_size=1)

        # rospy.Subscriber("/mid_grasp_point", Float32MultiArray, self.grasp_midpoint_callback)
        # rospy.Subscriber("joy", Joy, self.joy_cb)
        # rospy.Subscriber("final_grasp", Float32MultiArray, self.subscriber_callback)
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.fr_state_cb) 
        # rospy.Subscriber("/estimated_approach_frame", Float32MultiArray, self.l_shaped_callback) 
        rospy.sleep(1.0)
        rospy.Timer(rospy.Duration(0.033), self.timer_callback)


    def on_release(self):
        self.linear = [0,0,0]
        self.angular = [0,0,0]

    def _xyz_diff(self, start_pose, end_pose):

        difference = [0.0, 0.0, 0.0]
        difference[0] = end_pose[0] - start_pose[0]
        difference[1] = end_pose[1] - start_pose[1]
        difference[2] = end_pose[2] - start_pose[2]

        return difference

    def get_hiro_error_msg(self, grasp_quat):
        ret = []

        ret.append(self.error_state[0] * self.p_x)
        ret.append(self.error_state[1] * self.p_y)
        ret.append(self.error_state[2] * self.p_z)

        ret.append(grasp_quat[3])
        ret.append(grasp_quat[0])
        ret.append(grasp_quat[1])
        ret.append(grasp_quat[2])
        return ret
        
    
    def angle_error(self, gr, fr):
        gr_e = gr.as_euler("xyz", degrees=False)
        fr_e = fr.as_euler("xyz", degrees=False)

        gr_e = np.array(gr_e)
        fr_e = np.array(fr_e)      

        gr_e = gr_e / np.linalg.norm(gr_e)
        fr_e = fr_e / np.linalg.norm(fr_e) 

        ax = np.cross(gr_e, fr_e)
        ang = np.arctan2(np.linalg.norm(ax), np.dot(gr_e, fr_e))
        our_ang = so3.rotation(ax, ang)

        test_r = R.from_matrix(np.reshape(our_ang, newshape=[3,3]))
        output = test_r.as_euler("xyz", degrees=False)

        return output

    def calc_rotation_sign(self, fr_euler, gr_euler):
        fr_rot = R.from_euler("xyz", fr_euler, degrees=False)                   
        fr_quat = fr_rot.as_quat()
        fr_euler = fr_rot.as_euler('xyz', degrees=False)

        return fr_euler, fr_quat
    
    def quaterion_error(self, fr_quat, grasp_quat, fr_euler, grasp_euler):
        q1 = Quaternion(fr_quat[3], fr_quat[0], fr_quat[1], fr_quat[2])
        q2 = Quaternion(grasp_quat[3], grasp_quat[0], grasp_quat[1], grasp_quat[2])

        q_list = []
        for q in Quaternion.intermediates(q1, q2, 1, include_endpoints=True):
            rot = R.from_quat([q.unit.x, q.unit.y, q.unit.z, q.unit.w])
            euler = rot.as_euler('xyz', degrees=False)
            q_list.append(euler)

        if self.got_prev_sign:
            self.got_prev_sign = True
        else:
            for x in range(3):
                if (abs(grasp_euler[x] - fr_euler[x]) > abs(grasp_euler[x] + fr_euler[x])):
                    if (abs(self.prev_fr_euler[x] - fr_euler[x]) > abs(self.prev_fr_euler[x] + fr_euler[x])):
                        fr_euler[x] = -fr_euler[x]
        for x in range(3):
                self.prev_fr_euler[x] = fr_euler[x]
        return q_list[-1], fr_euler

    
    def lineseg_dist(self, p, a, b):
        p = np.array(p)
        a = np.array(a)
        b = np.array(b)
        d = np.divide(b - a, np.linalg.norm(b - a))

        s = np.dot(a - p, d)
        t = np.dot(p - b, d)

        h = np.maximum.reduce([s, t, 0])

        c = np.cross(p - a, d)

        return abs(np.hypot(h, np.linalg.norm(c)))
    
    def get_norm(self, a, b):
        a_np = np.array(a)
        b_np = np.array(b)

        return np.linalg.norm(b_np - a_np)
    
    def get_unit_line(self, a, b):
        a_np = np.asarray(a)
        b_np = np.asarray(b)

        line = b_np - a_np
        line = normalize([line], axis=1, norm='l1')
        return line[0]
    
    
    def find_ee_height(self, gripper_loc, x_a, grasp_loc):
        theta = np.rad2deg(np.arctan(self.cone_radius / self.cone_height))
        radius = self.lineseg_dist(gripper_loc, grasp_loc, x_a)

        hypot = self.get_norm(gripper_loc, grasp_loc)
        temp_theta = np.arcsin(radius/hypot)
        temp_theta_deg = np.rad2deg(temp_theta)
        new_height = np.cos(temp_theta) * hypot
        cone_radius = np.tan(theta) * new_height

        self.msg_obj_to_line = radius
        self.msg_cone_start = 1.0

        return new_height


    def check_collide(self, ee, ee_trans, ee_quat, cone, cone_trans, cone_quat):
        tf_ee = fcl.Transform(ee_quat, ee_trans)
        tf_cone = fcl.Transform(cone_quat, cone_trans)
        obj_ee = fcl.CollisionObject(ee, tf_ee)
        obj_cone = fcl.CollisionObject(cone, tf_cone)
        request = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True)
        result = fcl.DistanceResult()
        ret = fcl.distance(obj_ee, obj_cone, request, result)
        self.nearest_cone_points = result.nearest_points
        if ret > 0 and not self.grasp_loop.already_collided:
            return False
        else:
            self.grasp_loop.already_collided = True
            return True

    def wait_for_new_grasp(self):
        self.prev_grasp = self.grasp_pose[0]
        self.made_loop = False
        while True:
            if self.__check_for_new_grasp():
                return
            rospy.sleep(1)

    def __check_for_new_grasp(self):
        if self.prev_grasp != self.grasp_pose[0]:
            return True
        return False

    def move_through_list(self):
        if self.fr_state:
            if not self.made_loop:
                self.made_loop = True
                self.grasp_loop = GraspLoop(self.flag, self.grasp_pose, self.og_x_a)

            line = self.get_unit_line(self.grasp_pose[:3], self.og_x_a[:3])
            hiro_msg = Float64MultiArray()
            self.error_state = self.grasp_loop.get_curr_error(self.fr_position)
            # print(line)
            
            hiro_msg.data = self.get_hiro_error_msg(self.grasp_loop.grasp_dict["x_goal"][3:])

            hiro_msg.data[3] = self.grasp_loop.grasp_dict["x_goal"][6]
            hiro_msg.data[4] = self.grasp_loop.grasp_dict["x_goal"][3]
            hiro_msg.data[5] = self.grasp_loop.grasp_dict["x_goal"][4]
            hiro_msg.data[6] = self.grasp_loop.grasp_dict["x_goal"][5]

            for x in line:
                hiro_msg.data.append(x)
                
            hiro_msg.data.append(self.cone_radius)
            hiro_msg.data.append(self.cone_height)
            hiro_msg.data.append(self.msg_obj_to_line)
            hiro_msg.data.append(self.msg_cone_start)
            hiro_msg.data.append(self.og_x_a[0])
            hiro_msg.data.append(self.og_x_a[1])
            hiro_msg.data.append(self.og_x_a[2])
            hiro_msg.data.append(self.grasp_pose[0])
            hiro_msg.data.append(self.grasp_pose[1])
            hiro_msg.data.append(self.grasp_pose[2])

            self.hiro_ee_vel_goals_pub.publish(hiro_msg)
            self.on_release()
            if self.grasp_loop.check_next_state(self.error_state):
                # print("Over Here")
                rospy.signal_shutdown()
                # rospy.
                # self.wait_for_new_grasp()

    def timer_callback(self, event):
        # if self.flag == "linear":
        #     self.linear_movement()
        # elif self.flag == "xbox":
        #     self.xbox_input()
        # elif self.flag == "l-shaped":
        #     self.l_shaped_movement()
        # elif self.flag == "cone":
        #     self.cone()
        if self.flag == "list":
            self.move_through_list()

    def fr_state_cb(self, data):
        self.fr_position = [data.O_T_EE[12], data.O_T_EE[13], data.O_T_EE[14]]
        self.fr_state = True
        temp_rot_mat = np.array([
            [data.O_T_EE[0], data.O_T_EE[1], data.O_T_EE[2]],
            [data.O_T_EE[4], data.O_T_EE[5], data.O_T_EE[6]],
            [data.O_T_EE[8], data.O_T_EE[9], data.O_T_EE[10]]
        ])
        self.fr_rotation_matrix = temp_rot_mat
        
if __name__ == '__main__':
    # flag = rospy.get_param("/soup/flag")
    flag = "list"
    rospy.init_node('soup')
    xController = XboxInput(flag)
    rospy.spin()
