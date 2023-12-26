#!/usr/bin/python3

import rospy
import rospkg
from std_msgs.msg import Int8MultiArray, Float64MultiArray,  Float32MultiArray

def move_through_list(self):
        if self.fr_state:
            if not self.made_loop:
                self.made_loop = True
                self.grasp_loop = GraspLoop(self.flag, self.grasp_pose, self.og_x_a)

            line = self.get_unit_line(self.grasp_pose[:3], self.og_x_a[:3])
            hiro_msg = Float64MultiArray()
            self.error_state = self.grasp_loop.get_curr_error(self.fr_position)
            print(line)
            
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
                self.wait_for_new_grasp()