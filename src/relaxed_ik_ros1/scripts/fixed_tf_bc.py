#!/usr/bin/env python3
import roslib
from std_msgs.msg import Float32MultiArray
import rospy
import tf
from scipy.spatial.transform import Rotation as R
import numpy as np
import threading



class grasp_vis:
    def __init__(self):
        rospy.init_node('kinect_listener', anonymous=True)

        self.x = 0
        self.y = 0
        self.z = 0
        self.quat = [0.0, 0.0, 0.0, 1]
        self.br = tf.TransformBroadcaster()
        #self.t = tf.Transformer(False)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
        self.time = 0
        self.pub = rospy.Publisher("final_grasp", Float32MultiArray, queue_size=1)
        self.pub1 = rospy.Publisher("/estimated_approach_frame", Float32MultiArray, queue_size=1)
        self.pub2 = rospy.Publisher("/mid_grasp_point", Float32MultiArray, queue_size=1)
        self.mutex = threading.Lock()

        # print('reg')
        rospy.loginfo('rospy log')
        rospy.Timer(rospy.Duration(0.033), self.axis_callback)
        rospy.Subscriber("/grasp_net/grasp6d", Float32MultiArray, self.grasp_callback)
        rospy.spin()



    def axis_callback(self, event):
        trans, rot= None,None
        trans1, rot1 = None, None
        self.mutex.acquire()
        self.time = rospy.Time.now()
        self.br.sendTransform((0.816413, -0.00972354, 0.840409),
                         (0.680161, 0.707771, -0.1267, -0.142791),                        
                         self.time,
                         "kinect2_rgb_optical_frame",
                         "fr3_link0")
        
        self.br.sendTransform(
            (self.x, self.y, self.z),
            (self.quat[0], self.quat[1], self.quat[2], self.quat[3]),
            self.time,
            "grasp_frame",
            "kinect2_rgb_optical_frame"
        )
        r = R.from_euler('xyz', [0, 90, 0], degrees=True)
        quat_2 = r.as_quat()
        print(r)
        self.br.sendTransform(
            (0, 0, 0),
            (quat_2[0], quat_2[1], quat_2[2], quat_2[3]),
            self.time,
            "/final_grasp_frame",
            "grasp_frame"
        )

        self.br.sendTransform(
                (0, 0, -0.25),
                (0, 0, 0, 1),
                self.time,
                "/estimated_approach_frame",
                "final_grasp_frame"
            )
        
        mid_point = -0.25/2.0 + 0.022
        self.br.sendTransform(
                (0, 0, mid_point),
                (0, 0, 0, 1),
                self.time,
                "/mid_grasp_point",
                "final_grasp_frame"
            )
        
        try:
            self.listener.waitForTransform("/fr3_link0", "/final_grasp_frame", self.time, rospy.Duration(8.0))
            (trans, rot) = self.listener.lookupTransform(
                "/fr3_link0", 
                "/final_grasp_frame", 
                self.time
            )    
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) :
            pass

        msg = Float32MultiArray()
        msg.data = np.concatenate((list(trans), list(rot)))
        self.pub.publish(msg)
        
        try:
            self.listener.waitForTransform("/fr3_link0", "/estimated_approach_frame", self.time, rospy.Duration(8.0))
            (trans1, rot1) = self.listener.lookupTransform(
                "/fr3_link0",
                "/estimated_approach_frame",
                self.time
            )
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
            pass

        try:
            self.listener.waitForTransform("/fr3_link0", "/mid_grasp_point", self.time, rospy.Duration(4.0))
            (trans2, rot2) = self.listener.lookupTransform(
                "/fr3_link0",
                "/mid_grasp_point",
                self.time
            )
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
            pass
        
        msg1 = Float32MultiArray()
        msg1.data = np.concatenate((list(trans1), list(rot1)))
        grasp_r = R.from_quat(msg1.data[3:])
        grasp_euler = grasp_r.as_euler('xyz', degrees=False)
        print(grasp_euler)
        self.pub1.publish(msg1)


        msg2 = Float32MultiArray()
        msg2.data = np.concatenate((list(trans2), list(rot2)))
        self.pub2.publish(msg2)
            

        self.mutex.release()

    def grasp_callback(self, data):
        self.mutex.acquire()
        print('*********')
        # rospy.loginfo(data.data)
        self.x = data.data[0]
        self.y = data.data[1]
        self.z = data.data[2]
        print("XYZ", self.x, self.y, self.z)
        rot_mat = data.data[3:]
        rot_mat = np.reshape(rot_mat, (3,3))
        r = R.from_matrix(rot_mat)
        
        self.quat = r.as_quat()
        # print("quad", self.quat)
        self.mutex.release()

if __name__ == '__main__':

    grasp_vis()