#!/usr/bin/env python

import rospy
import marshal
import datetime
import math
import tf
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker


def gpsCallback(data):
	gps_reading = marshal.loads(data.data)
	current_time = rospy.Time.now()
	frame_id = "gps_link"

	time_ref_msg = TimeReference()
	time_ref_msg.header.stamp = current_time
	time_ref_msg.header.frame_id = frame_id
	# if 'timestamp' in gps_reading:
	# 	timestamp = gps_reading['timestamp']
	# 	timestamp_s = datetime.time(
	# 		hour=int(timestamp[0:2]),
	# 		minute=int(timestamp[3:5]),
	# 		second=int(timestamp[6:8]),
	# 		microsecond=int(timestamp[9:]))
	# 	time_ref_msg.time_ref = rospy.Time.from_sec(timestamp_s.second)
	# 	time_ref_msg.source = "gps_time"
	# else:
	# 	time_ref_msg.source = frame_id
	time_ref_msg.source = frame_id

	time_ref_pub.publish(time_ref_msg)

	nav_msg = NavSatFix()
	nav_msg.header.stamp = current_time
	nav_msg.header.frame_id = frame_id

	gps_qual = gps_reading['qual']

	if gps_qual == 1:
		nav_msg.status.status = NavSatStatus.STATUS_FIX
	elif gps_qual == 2:
		nav_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
	elif gps_qual in (4, 5):
		nav_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
	elif gps_qual == 9:
		nav_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
	else:
		nav_msg.status.status = NavSatStatus.STATUS_NO_FIX

	nav_msg.status.service = NavSatStatus.SERVICE_GPS

	nav_msg.latitude = gps_reading['latitude']
	nav_msg.longitude = gps_reading['longitude']
	# nav_msg.altitude = float('NaN')
	nav_msg.altitude = 0 # EKF Not outputing when using NaN? 
	nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

	navsatfix_pub.publish(nav_msg)

	vel_msg = TwistStamped()
	vel_msg.header.stamp = current_time
	vel_msg.header.frame_id = frame_id
	vel_msg.twist.linear.x = gps_reading['speed_over_ground'] * math.sin(gps_reading['course_over_ground'])
	vel_msg.twist.linear.y = gps_reading['speed_over_ground'] * math.cos(gps_reading['course_over_ground'])

	vel_pub.publish(vel_msg)

	marker_msg = Marker()
	marker_msg.header = nav_msg.header
	marker_msg.action = 0 # ADD
	marker_msg.type = 0 # ARROW
	marker_msg.scale.x = 1
	marker_msg.scale.y = 0.1
	marker_msg.scale.z = 0.1
	marker_msg.color.a = 1.0
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 0.0;
	marker_msg.color.b = 1.0;
	marker_msg.pose.position.x = 0
	marker_msg.pose.position.y = 0
	quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	marker_msg.pose.orientation.x = quat[0]
	marker_msg.pose.orientation.y = quat[1]
	marker_msg.pose.orientation.z = quat[2]
	marker_msg.pose.orientation.w = quat[3]
	
	marker_pub.publish(marker_msg)



if __name__ == '__main__':

	rospy.init_node('gpsTranslator')

	rospy.Subscriber('gps/dict', String, gpsCallback)

	navsatfix_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
	time_ref_pub = rospy.Publisher('gps/timeref', TimeReference, queue_size=10)
	vel_pub = rospy.Publisher('gps/vel', TwistStamped, queue_size=10)
	marker_pub = rospy.Publisher('gps/marker', Marker, queue_size=10)

	rospy.spin()

