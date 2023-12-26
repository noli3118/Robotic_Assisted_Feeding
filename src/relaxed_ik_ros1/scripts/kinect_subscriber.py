#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
import tf

def callback(data):
    rospy.loginfo(data.data)
    publish_marker(data.data)

    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('kinect_listener', anonymous=True)
    
    # tf_listener = tf.TransformListener()
    # print(tf_listener.allFramesAsString())
    # (trans, rot) = tf_listener.lookupTransform("fr3_link0", "fr3_link1", rospy.Time())
    # rospy.loginfo('Translation of obj', trans)

    rospy.Subscriber("/grasp_net/grasp6d", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def publish_marker(marker_pose, radius=0.05):
    kinect_marker_pub = rospy.Publisher("/grasp_pose_location", Marker, queue_size = 1)

    marker = Marker()
    camera_translation = [0.491956, 0.116437, 0.586158]
    marker.pose.position.x = marker_pose[0] + camera_translation[0]
    marker.pose.position.y = marker_pose[1] + camera_translation[1]
    marker.pose.position.z = marker_pose[2] - camera_translation[2]

    marker.header.frame_id = "fr3_link0"
    marker.type = Marker.SPHERE
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.header.stamp = rospy.Time()
    marker.ns = ""
    marker.action = Marker.ADD

    marker.id = 1

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.scale.x = radius
    marker.scale.y = radius
    marker.scale.z = radius

    kinect_marker_pub.publish(marker)

if __name__ == '__main__':
    listener()

    rospy.init_node('static_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br.sendTransform((0.491956, 0.116437, 0.586158),
                         (0.691802, 0.72111, -0.0333233, 0.01731),
                         rospy.Time.now(),
                         "kinect2_rgb_optical_frame",
                         "fr3_link0")