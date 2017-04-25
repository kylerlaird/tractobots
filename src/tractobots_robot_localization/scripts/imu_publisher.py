#!/usr/bin/env python
# Node for changing frame of IMU msg when working with a rosbag file and 
# publishing a marker for rviz visualization.

import rospy
import tf
from math import pi
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

def imuCallback(data):
	imu_msg = Imu()
	# imu_msg = data
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.header.frame_id = "imu_link"
	quat = (
    data.orientation.x,
    data.orientation.y,
    data.orientation.z,
    data.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quat)
	quat2 = tf.transformations.quaternion_from_euler(0, 0, euler[0]+pi)
	imu_msg.orientation.x = quat2[0]
	imu_msg.orientation.y = quat2[1]
	imu_msg.orientation.z = quat2[2]
	imu_msg.orientation.w = quat2[3]

	imu_pub.publish(imu_msg)

	marker_msg = Marker()
	marker_msg.header = imu_msg.header
	marker_msg.action = 0 # ADD
	marker_msg.type = 0 # ARROW
	marker_msg.scale.x = 1
	marker_msg.scale.y = 0.1
	marker_msg.scale.z = 0.1
	marker_msg.color.a = 1.0
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 1.0;
	marker_msg.color.b = 0.0;
	marker_msg.pose.position.x = 0
	marker_msg.pose.position.y = 0
	marker_msg.pose.orientation.x = imu_msg.orientation.x
	marker_msg.pose.orientation.y = imu_msg.orientation.y
	marker_msg.pose.orientation.z = imu_msg.orientation.z
	marker_msg.pose.orientation.w = imu_msg.orientation.w
    
	marker_pub.publish(marker_msg)

if __name__ == '__main__':

	rospy.init_node('imuPublisher')

	rospy.Subscriber('/tfsensors/imu1', Imu, imuCallback)

	imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
	marker_pub = rospy.Publisher('imu/marker', Marker, queue_size=10)
	
	rospy.spin()

