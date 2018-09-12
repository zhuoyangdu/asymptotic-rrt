#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

import cv2
from cv_bridge import CvBridge

print "Please input init state of the vehicle: x y theta"
string_state = raw_input()
[x,y,theta] = string_state.split(" ")

if __name__=="__main__":
    rospy.init_node("pub_states")
    pub_map = rospy.Publisher("/vrep/map", Image, queue_size=10)
    pub_state = rospy.Publisher("/vrep/vehicle_state", PoseStamped, queue_size=10)
    rate = rospy.Rate(1)

    path = "./map/map.bmp"
    img = cv2.imread(path)
    ros_img = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    state = PoseStamped()
    state.header.stamp = rospy.get_rostime()
    state.pose.position.x = float(x)
    state.pose.position.y = float(y)
    q = quaternion_from_euler(float(theta), 0, 0)
    state.pose.orientation.x = q[0]
    state.pose.orientation.y = q[1]
    state.pose.orientation.z = q[2]
    state.pose.orientation.w = q[3]

    while not rospy.is_shutdown():
        pub_map.publish(ros_img)
        pub_state.publish(state)
        print "publish state: ", state
        rate.sleep()
