#!/usr/bin/env python

import rospy
import tf2_msgs.msg
import geometry_msgs.msg

def publish_tf():
    rospy.init_node('tf_publisher')

    tf_pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Get the current time
        current_time = rospy.Time.now()

        # Create first transform (map -> base_link)
        transform_1 = geometry_msgs.msg.TransformStamped()
        transform_1.header.stamp = current_time
        transform_1.header.frame_id = "map"
        transform_1.child_frame_id = "base_link"
        transform_1.transform.translation.x = 1.8657469520840229
        transform_1.transform.translation.y = -0.9633663605164087
        transform_1.transform.translation.z = -0.09614687592355328
        transform_1.transform.rotation.x = 0.6598240544141581
        transform_1.transform.rotation.y = -0.10548286114364468
        transform_1.transform.rotation.z = -0.023369535851761904
        transform_1.transform.rotation.w = 0.74361243132124

        # Create second transform (map -> goal)
        transform_2 = geometry_msgs.msg.TransformStamped()
        transform_2.header.stamp = current_time
        transform_2.header.frame_id = "map"
        transform_2.child_frame_id = "goal"
        transform_2.transform.translation.x = -3.798898220062256
        transform_2.transform.translation.y = 0.1770249456167221
        transform_2.transform.translation.z = -0.20741093158721924
        transform_2.transform.rotation.x = 0.21383614658569156
        transform_2.transform.rotation.y = 0.7333274507132186
        transform_2.transform.rotation.z = -0.6282281385078758
        transform_2.transform.rotation.w = 0.1477645371216094

        # Publish the transforms
        tf_message = tf2_msgs.msg.TFMessage([transform_1, transform_2])
        tf_pub.publish(tf_message)

        print("TF Published")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_tf()
    except rospy.ROSInterruptException:
        pass
