#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class PoseTransformer:

    def __init__(self, from_frame, to_frame):

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))  # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
                transform = tf_buffer.lookup_transform(to_frame,
                                                       from_frame,  # source frame
                                                       rospy.Time(0),  # get the tf at first available time
                                                       rospy.Duration(0.1))  # wait for 1 second

                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

                pose_pub.publish(pose_transformed)

            except Exception as e:
                rospy.logwarn(e)
                continue

if __name__ == '__main__':
    rospy.init_node('pose_transformer')
    from_frame = rospy.get_param('~from_frame')
    to_frame = rospy.get_param('~to_frame')
    pose_pub = rospy.Publisher('pose_example_publisher', PoseStamped, queue_size=10)
    ptf = PoseTransformer(from_frame, to_frame)
    rospy.loginfo("Transforming pose from frame: %s to frame: %s" % (from_frame, to_frame))

    rospy.spin()
