#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tf_trans
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage

# Node to subscribe to the aruco_marker_frame's pose wrt to the base_link of the drone
# And invert it to publish the drone's base_link pose wrt to the aruco marker

def camera_info_callback(camera_info_msg):
    # Update the D matrix with the new values
    camera_info_msg.D = [0.0355633167278393, -0.123584193169489, -0.000250953321849423, -0.000181034148553509, 0.155155222181198]
    camera_info_msg.P = [2992.533447265625, 0.0, 1917.8743896484375, 0.0, 0.0, 2994.665771484375, 1038.7142333984375, 0.0, 0.0, 0.0, 1.0, 0.0]

    # Publish the modified CameraInfo message to a new topic
    hl2_camera_info_pub.publish(camera_info_msg)

def pose_callback(msg):
    # Get the pose from the message
    marker_pose = msg.pose

    # Extract the position and orientation
    marker_position = (marker_pose.position.x, marker_pose.position.y, marker_pose.position.z)
    marker_orientation = (marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w)

    # Convert to transformation matrix
    marker_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(marker_position),
        tf_trans.quaternion_matrix(marker_orientation)
    )

    # Invert the transformation matrix
    inverted_transform = tf_trans.inverse_matrix(marker_transform)

    # Extract the inverted translation and rotation
    inverted_translation = tf_trans.translation_from_matrix(inverted_transform)
    inverted_rotation = tf_trans.quaternion_from_matrix(inverted_transform)

    # Publish the inverted transformation as a PoseStamped message
    inverted_pose = PoseStamped()
    inverted_pose.header.stamp = msg.header.stamp  # Use the original timestamp
    inverted_pose.header.frame_id = "aruco_marker_frame"
    inverted_pose.pose.position.x = inverted_translation[0]
    inverted_pose.pose.position.y = inverted_translation[1]
    inverted_pose.pose.position.z = inverted_translation[2]
    inverted_pose.pose.orientation.x = inverted_rotation[0]
    inverted_pose.pose.orientation.y = inverted_rotation[1]
    inverted_pose.pose.orientation.z = inverted_rotation[2]
    inverted_pose.pose.orientation.w = inverted_rotation[3]

    inverted_pose_pub.publish(inverted_pose)

def hl2_pose_callback(msg):
    # Get the pose from the message
    hl2_pose = msg.pose

    # Extract the position and orientation
    hl2_position = (hl2_pose.position.x, hl2_pose.position.y, hl2_pose.position.z)
    hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)

    # Publish the transform as a TransformStamped message
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = msg.header.stamp  # Use the original timestamp
    transform_stamped.header.frame_id = "map"
    transform_stamped.child_frame_id = "HL2_camera"
    transform_stamped.transform.translation.x = hl2_position[0]
    transform_stamped.transform.translation.y = hl2_position[1]
    transform_stamped.transform.translation.z = hl2_position[2]
    transform_stamped.transform.rotation.x = hl2_orientation[0]
    transform_stamped.transform.rotation.y = hl2_orientation[1]
    transform_stamped.transform.rotation.z = hl2_orientation[2]
    transform_stamped.transform.rotation.w = hl2_orientation[3]

    tf_msg = TFMessage([transform_stamped])
    tf_pub.publish(tf_msg)

def goal_pose_callback(msg):
    # Get the pose from the message
    goal_pose = msg.pose

    # Extract the position and orientation
    goal_position = (goal_pose.position.x, goal_pose.position.y, goal_pose.position.z)
    goal_orientation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)

    # Publish the transform as a TransformStamped message
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = msg.header.stamp  # Use the original timestamp
    transform_stamped.header.frame_id = "map"
    transform_stamped.child_frame_id = "goal"
    transform_stamped.transform.translation.x = goal_position[0]
    transform_stamped.transform.translation.y = goal_position[1]
    transform_stamped.transform.translation.z = goal_position[2]
    transform_stamped.transform.rotation.x = goal_orientation[0]
    transform_stamped.transform.rotation.y = goal_orientation[1]
    transform_stamped.transform.rotation.z = goal_orientation[2]
    transform_stamped.transform.rotation.w = goal_orientation[3]

    tf_msg = TFMessage([transform_stamped])
    tf_pub.publish(tf_msg)

def aruco_drone_pose_callback(msg):
    # Invert the pose for base_link -> aruco_marker_drone_frame and publish it
    marker_pose = msg.pose
    marker_position = (marker_pose.position.x, marker_pose.position.y, marker_pose.position.z)
    marker_orientation = (marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w)
    
    marker_transform = tf_trans.concatenate_matrices(
        tf_trans.translation_matrix(marker_position),
        tf_trans.quaternion_matrix(marker_orientation)
    )
    
    inverted_transform = tf_trans.inverse_matrix(marker_transform)
    inverted_translation = tf_trans.translation_from_matrix(inverted_transform)
    inverted_rotation = tf_trans.quaternion_from_matrix(inverted_transform)
    
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = msg.header.stamp
    transform_stamped.header.frame_id = 'aruco_marker_frame'
    transform_stamped.child_frame_id = 'base_link'
    transform_stamped.transform.translation.x = inverted_translation[0]
    transform_stamped.transform.translation.y = inverted_translation[1]
    transform_stamped.transform.translation.z = inverted_translation[2]
    transform_stamped.transform.rotation.x = inverted_rotation[0]
    transform_stamped.transform.rotation.y = inverted_rotation[1]
    transform_stamped.transform.rotation.z = inverted_rotation[2]
    transform_stamped.transform.rotation.w = inverted_rotation[3]

    tf_msg = TFMessage([transform_stamped])
    tf_pub.publish(tf_msg)

def aruco_hl2_pose_callback(msg):
    # Rename aruco_marker_hl2_frame to aruco_marker_frame and publish it
    hl2_pose = msg.pose
    hl2_position = (hl2_pose.position.x, hl2_pose.position.y, hl2_pose.position.z)
    hl2_orientation = (hl2_pose.orientation.x, hl2_pose.orientation.y, hl2_pose.orientation.z, hl2_pose.orientation.w)
    
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = msg.header.stamp
    transform_stamped.header.frame_id = 'HL2_camera'
    transform_stamped.child_frame_id = 'aruco_marker_frame'
    transform_stamped.transform.translation.x = hl2_position[0]
    transform_stamped.transform.translation.y = hl2_position[1]
    transform_stamped.transform.translation.z = hl2_position[2]
    transform_stamped.transform.rotation.x = hl2_orientation[0]
    transform_stamped.transform.rotation.y = hl2_orientation[1]
    transform_stamped.transform.rotation.z = hl2_orientation[2]
    transform_stamped.transform.rotation.w = hl2_orientation[3]
    
    tf_msg = TFMessage([transform_stamped])
    tf_pub.publish(tf_msg)

if __name__ == '__main__':
    rospy.init_node('drone_pose_with_respect_to_marker')

    # Enable use_sim_time parameter
    rospy.set_param('use_sim_time', False)

    hl2_camera_info_pub = rospy.Publisher('/Player0/camera/camera_info/updated', CameraInfo, queue_size=10)
    rospy.Subscriber('/Player0/camera/camera_info', CameraInfo, camera_info_callback) 
    
    inverted_pose_pub = rospy.Publisher('/inverted_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('/aruco_drone/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/aruco_hl2/pose', PoseStamped, aruco_hl2_pose_callback)
    
    tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
    rospy.Subscriber('/Player0/camera/pose', PoseStamped, hl2_pose_callback)
    rospy.Subscriber('/tello/command_pos_world', PoseStamped, goal_pose_callback)

    rospy.spin()
