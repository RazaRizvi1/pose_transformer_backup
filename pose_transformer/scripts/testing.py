#!/usr/bin/env python

import rospy
import tf
import math
import geometry_msgs.msg
import tf.transformations as tf_trans
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage

## Node that DOES changes the frame convention at all!

# Variables to store transforms internally
aruco_drone_transform = None
aruco_hl2_transform = None
hl2_pose_transform = None
goal_pose_transform = None

tf_broadcaster = tf.TransformBroadcaster()

def aruco_drone_callback(msg):
    hl2_pose = msg.pose

    # Extract position and orientation
    hl2_position = (hl2_pose.position.x, hl2_pose.position.y, hl2_pose.position.z)
    hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)

    # Compute the inverse translation and rotation
    inverse_translation = tf_trans.quaternion_matrix(hl2_orientation).dot([-hl2_position[0], -hl2_position[1], -hl2_position[2], 1.0])[:3]
    inverse_rotation = tf_trans.quaternion_conjugate(hl2_orientation)

    # Broadcast the inverse transform
    tf_broadcaster.sendTransform(
        inverse_translation,
        inverse_rotation,
        msg.header.stamp,  # Use the timestamp from the message
        "base_link",       # Child frame (inverted)
        "aruco"            # Parent frame (inverted)
    )

    print("base_link-aruco transform broadcasted")
# Subscribes to the /tello/command_pos_world topic
# Saves the map -> goal transform
# Follows the ROS frame convention already
# def goal_pose_callback(msg):
#     goal_pose = msg.pose

#     # Extract position and orientation
#     goal_position = (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)
#     goal_orientation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)

#     # Broadcast the transform
#     tf_broadcaster.sendTransform(
#         goal_position,
#         goal_orientation,
#         rospy.Time.now(),
#         "goal",        # Child frame
#         "map"          # Parent frame
#     )

#     print("map-goal transform broadcasted")

# Subscribes to the aruco_hl2 pose topic
# Saves the hl2 -> aruco_marker_frame transform

if __name__ == '__main__':
    rospy.init_node('aruco_wrt_base_link')

    # rospy.set_param('use_sim_time', True)
    
    # rospy.Subscriber('/aruco_drone/pose', PoseStamped, aruco_drone_pose_callback)
    # rospy.Subscriber('/aruco_hl2/pose', PoseStamped, aruco_aruco_drone_callback)
    
    rospy.Subscriber('/aruco_drone/pose', PoseStamped, aruco_drone_callback)
    # rospy.Subscriber('/tello/command_pos_world', PoseStamped, goal_pose_callback)

    tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

    # Create a timer to call the publish_transforms method periodically (e.g., every 0.1 seconds)
    # rospy.Timer(rospy.Duration(1.0), publish_transforms)

    rospy.spin()
