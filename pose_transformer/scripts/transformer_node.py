#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tf_trans
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage


# This node is now a copy of new_transformer_node.py

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
    inverted_pose.header.stamp = rospy.Time.now()
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
    transform_stamped.header.stamp = msg.header.stamp
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
    new_tf_pub.publish(tf_msg)

class TFListener:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.new_tf_pub = rospy.Publisher('/new_tf', TFMessage, queue_size=10)
        self.transforms = {
            'world': 'tello_camera_odom',
            'base_link': 'aruco_marker_drone_frame',
            'HL2_camera': 'aruco_marker_hl2_frame'
        }
        self.rate = rospy.Rate(10.0)  # 10 Hz

    def listen_and_publish_transforms(self):
        while not rospy.is_shutdown():
            tf_msg = TFMessage()
            for source, target in self.transforms.items():
                try:
                    (trans, rot) = self.listener.lookupTransform(source, target, rospy.Time(0))
                    if target == 'aruco_marker_drone_frame':
                        # Invert transform for base_link -> aruco_marker_drone_frame and rename
                        inverted_trans, inverted_rot = self.invert_transform(trans, rot)
                        transform_stamped = TransformStamped()
                        transform_stamped.header.stamp = rospy.Time.now()
                        transform_stamped.header.frame_id = 'aruco_marker_frame'
                        transform_stamped.child_frame_id = 'base_link'
                        transform_stamped.transform.translation.x = inverted_trans[0]
                        transform_stamped.transform.translation.y = inverted_trans[1]
                        transform_stamped.transform.translation.z = inverted_trans[2]
                        transform_stamped.transform.rotation.x = inverted_rot[0]
                        transform_stamped.transform.rotation.y = inverted_rot[1]
                        transform_stamped.transform.rotation.z = inverted_rot[2]
                        transform_stamped.transform.rotation.w = inverted_rot[3]
                        tf_msg.transforms.append(transform_stamped)
                    elif target == 'aruco_marker_hl2_frame':
                        # Rename aruco_marker_hl2_frame to aruco_marker_frame
                        transform_stamped = TransformStamped()
                        transform_stamped.header.stamp = rospy.Time.now()
                        transform_stamped.header.frame_id = 'HL2_camera'
                        transform_stamped.child_frame_id = 'aruco_marker_frame'
                        transform_stamped.transform.translation.x = trans[0]
                        transform_stamped.transform.translation.y = trans[1]
                        transform_stamped.transform.translation.z = trans[2]
                        transform_stamped.transform.rotation.x = rot[0]
                        transform_stamped.transform.rotation.y = rot[1]
                        transform_stamped.transform.rotation.z = rot[2]
                        transform_stamped.transform.rotation.w = rot[3]
                        tf_msg.transforms.append(transform_stamped)
                    else:
                        # Publish as it is for world -> tello_camera_odom
                        transform_stamped = TransformStamped()
                        transform_stamped.header.stamp = rospy.Time.now()
                        transform_stamped.header.frame_id = 'world'
                        transform_stamped.child_frame_id = 'tello_camera_odom'
                        transform_stamped.transform.translation.x = trans[0]
                        transform_stamped.transform.translation.y = trans[1]
                        transform_stamped.transform.translation.z = trans[2]
                        transform_stamped.transform.rotation.x = rot[0]
                        transform_stamped.transform.rotation.y = rot[1]
                        transform_stamped.transform.rotation.z = rot[2]
                        transform_stamped.transform.rotation.w = rot[3]
                        tf_msg.transforms.append(transform_stamped)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn(f"Could not find transform from {source} to {target}")

            self.new_tf_pub.publish(tf_msg)
            self.rate.sleep()

    def invert_transform(self, translation, rotation):
        transform_matrix = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(translation),
            tf_trans.quaternion_matrix(rotation)
        )
        inverted_matrix = tf_trans.inverse_matrix(transform_matrix)
        inverted_translation = tf_trans.translation_from_matrix(inverted_matrix)
        inverted_rotation = tf_trans.quaternion_from_matrix(inverted_matrix)
        return inverted_translation, inverted_rotation

if __name__ == '__main__':
    rospy.init_node('drone_pose_with_respect_to_marker')

    hl2_camera_info_pub = rospy.Publisher('/Player0/camera/camera_info/updated', CameraInfo, queue_size=10)
    rospy.Subscriber('/Player0/camera/camera_info', CameraInfo, camera_info_callback) 
    
    inverted_pose_pub = rospy.Publisher('/inverted_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('/aruco_drone/pose', PoseStamped, pose_callback)
    
    new_tf_pub = rospy.Publisher('/new_tf', TFMessage, queue_size=10)
    rospy.Subscriber('/Player0/camera/pose', PoseStamped, hl2_pose_callback)

    tf_listener = TFListener()
    tf_listener.listen_and_publish_transforms()

    rospy.spin()
