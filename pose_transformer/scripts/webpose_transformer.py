#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Transform, PoseStamped, TransformStamped, Point, Quaternion
from utils import *

class Node:

    def __init__(self):

        # initialize parameters
        rospy.init_node('webinspector_pose_visualizer')

        self.pub = rospy.Publisher('/tello/global_pos', PoseStamped, queue_size=1)

        rospy.Subscriber('/webinspector_pose', Pose, self.callback_command)

        rospy.spin() 

    def callback_command(self, msg):
        
        pq = unpack_pose(msg)
        T_m2_c2 = pq2matrix(pq)
        T_link_c2 = T_inv(pose2matrix([0,0,0,-0.5,0.5,-0.5,0.5]))
        T_m2_link = T_m2_c2
        T_m2_c2 = T_m2_link.dot(T_link_c2) # Convert ENU to camera frame
        ps = create_pose(T_m2_c2, 'world')

        self.pub.publish(ps)

if __name__ == '__main__':
    Node()
    
