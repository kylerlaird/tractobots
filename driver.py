#!/usr/bin/env python

import rospy
#from sensor_msgs.msg import Joy
import sensor_msgs.msg
import std_msgs.msg

import random
import sys
import signal 

import time

import threading

import logging

import marshal

import math
import PID
import numpy
import nvector

max_stamp_age = rospy.Duration.from_sec(2)

north = 0.0
#north = 358.0
#north = 359.7

#logging.basicConfig(level=logging.DEBUG, format='(%(threadName)-10s) %(message)s')
logging.basicConfig(level=logging.INFO, format='(%(threadName)-10s) %(message)s')

def minmax(min, max, i):
        if i < min:
		return(min)
	if i > max:
		return(max)
	return(i)

def reverse_course(course):
	return((course + 180.) % 360)


class Scaler():
	def __init__(self, a_min, a_max, b_min, b_max):
		def f(x):
			return((x - a_min)/(a_max - a_min) * (b_max - b_min) + b_min)

		self.__call__ = f


if False:
	s = Scaler(0, 1, 100, 0)

	for i in range(0, 10):
		j = i / 10.
		print j, s(j)

	sys.exit(0)

		
class Setting_Publisher():
	def __init__(self, node_name, scaler):
		self.node_name = node_name
		self.scaler = scaler
		self.pub = rospy.Publisher(self.node_name, std_msgs.msg.UInt8, queue_size=1)

	def __call__(self, value):
		v = int(self.scaler(float(value)))
		#print self.node_name, value, v
		self.pub.publish(v)

class Line_Follower():
	name = 'Line Follower'


	def __init__(self, name=None):
		if name:
			self.name = name

		print 'starting %s' % (self)

		self.captured = 0
		self.captured_distance = 0.50
		
		self.offset = 0.0

		self.throttle = 0.50

		# 000.027, 000.018 at 7 MPH!
		#self.xtd_pid = PID.PID(P=12.0, I=1.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)
		# 000.020, 000.017 at 7
		#self.pid_captured = PID.PID(P=20.0, I=1.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)


		# Used for drilling
		#self.pid_captured = PID.PID(P=20.0, I=8.0, D=4.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)
		#self.pid_seeking = PID.PID(P=6.0, I=0.0, D=4.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)

		# Used for cart.
		self.pid_captured = PID.PID(P=100.0, I=70.0, D=50, Derivator=0, Integrator=0, Integrator_max=50, Integrator_min=-50)
		self.pid_seeking = PID.PID(P=10.0, I=0.0, D=4.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)

		# Yikes!  Hunting...
		#self.pid_captured = PID.PID(P=30.0, I=30.0, D=10.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)
		#self.pid_seeking = PID.PID(P=30.0, I=0.0, D=4.0, Derivator=0, Integrator=0, Integrator_max=1, Integrator_min=-1)


		self.frame = nvector.FrameE(name='WGS84')

		self.pub_xtd = rospy.Publisher('/navigation/xtd', std_msgs.msg.Float64, queue_size=10)
		
	def __str__(self):
		return(self.name)

        def load_line(self, pointA, pointB):
                self.path = nvector.GeoPath(pointA, pointB)
                # Ick.  This looks horrible.
                p_AB_E = nvector.diff_positions(pointA, pointB)
                frame_N = nvector.FrameN(pointA)
                p_AB_N = p_AB_E.change_frame(frame_N)
                p_AB_N = p_AB_N.pvector.ravel()
                azimuth = numpy.arctan2(p_AB_N[1], p_AB_N[0])
                self.course = numpy.rad2deg(azimuth) % 360
                #print 'azimuth:', azimuth, self.course
                self.course_distance = self.path.positionB.distance_and_azimuth(self.path.positionA)[0]

		self.offset = 0.0

	def load_vector(self, pointA, azimuth):
		pointB, _azimuth2 = pointA.geo_point(distance=1000, azimuth=azimuth, degrees=True)
		print pointA
		return(self.load_line(pointA, pointB))

	def flip_line(self):
		self.load_line(self.path.positionB, self.path.positionA)

	def __call__(self, gps_update):
		position = self.frame.GeoPoint(
			latitude=gps_update['latitude'],
			longitude=gps_update['longitude'],
			degrees=True,
		)

		# How far off the line are we?
		self.cross_track_distance = self.path.cross_track_distance(position, method='greatcircle').ravel() - self.offset
		print 'XTD:%0.2f (offset:%0.2f)' % (self.cross_track_distance, self.offset),

		self.pub_xtd.publish(self.cross_track_distance)
	
		speed_over_ground = gps_update['speed_over_ground']
		course_over_ground = gps_update['course_over_ground']
		heading = gps_update['heading']


		# How far off the line will we be?
		# 000.027, 000.018
		if False:
			#self.position_guess = position.geo_point(distance=speed_over_ground * 0.001, azimuth=course_over_ground, degrees=True)[0]	
			self.position_guess = position.geo_point(distance=speed_over_ground * 0.01, azimuth=course_over_ground, degrees=True)[0]	
			self.cross_track_distance_future = self.path.cross_track_distance(self.position_guess, method='greatcircle').ravel()
			print 'XTDf:%0.2f' % (self.cross_track_distance_future),


			pid_feed = self.cross_track_distance_future
		else:
			pid_feed = self.cross_track_distance

		if abs(self.cross_track_distance) <= self.captured_distance:
			self.captured += 1
		else:
			self.captured = 0
			
	
		if self.captured > 15:
			print 'captured'
			pid_output = self.pid_captured(pid_feed)
		else:	
			pid_output = self.pid_seeking(pid_feed)
			
		pid_output = self.pid_seeking(pid_feed)

		#desired_heading = (self.course + minmax(-90, 90, self.xtd_pid.update(pid_feed))) % 360
		desired_heading = (self.course + minmax(-90, 90, pid_output)) % 360

		print 'crs:%d' % (self.course),
		print 'dhdg:%d' % (desired_heading),

		heading_correction = +0
		bearing = (desired_heading - heading + heading_correction)

		if bearing < -180:
			bearing += 360
		elif bearing > 180:
			bearing -= 360

		#print self.course, gps_update['heading'], bearing,
		print '****', gps_update['heading']

		solution = {
			'bearing': bearing,
		}

		if False and abs(bearing) > 15:
			solution['transmission'] = 0
		else:
			solution['transmission'] = +1


		# For now, always use set amount of throttle.
		#solution['throttle'] = 0.50

		solution['throttle'] = self.throttle

		return(solution)


class Compass_Follower():
	name = 'Compass Follower'

	def __init__(self, course):
		self.course = course

	def __str__(self):
		return('%s: %d' % (self.name, self.course))

	def __call__(self, gps_update):
		bearing = self.course - gps_update['heading']
		if bearing < -180:
			bearing += 360
		elif bearing > 180:
			bearing -= 360

		#print self.course, gps_update['heading'], bearing,
		print '****', gps_update['heading']

		solution = {
			'bearing': bearing,
		}

		if False and abs(bearing) > 15:
			solution['transmission'] = 0
		else:
			solution['transmission'] = +1

		return(solution)

class MT765():
	def __init__(self, name):
		self.name = name
		self.frame = nvector.FrameE(name='WGS84')
		#self.adc_lock = threading.Lock()
		self.timeout_neutral = 2
		self.timeout_kill = 10
		self._enabled = False

		self.gps_reading = None
		self.joy_data_prev = None

		#self.steering = Setting_Publisher('/steering/setting', Scaler(+1, -1, 64, 192))
		##self.steering = Setting_Publisher('/steering/setting', Scaler(+1, -1, 96, 160))
		#self.transmission = Setting_Publisher('/transmission/setting', Scaler(+1, -1, 64, 192))

		# 2016.10.10
		self.steering = Setting_Publisher('/steering/put', Scaler(+1, -1, 104, 152))
		self.transmission = Setting_Publisher('/transmission/put', Scaler(+1, -1, 64, 192))
		self.throttle = Setting_Publisher('/throttle/put', Scaler(+1, 0, 20, 230))


		if False:
			for i in range(-10, 10):
				j = i / 10.
				self.steering(j)

			self.steering(0)
			# Stop...
			sys.exit()



		#self.steer_for_bearing = PID.PID(P=0.01, I=0.02, D=0.01, Derivator=0, Integrator=0, Integrator_max=2, Integrator_min=-2, output_min = -1.0, output_max=+1.0)
		self.steer_for_bearing = PID.PID(P=0.02, I=0.02, D=0.02, Derivator=0, Integrator=0, Integrator_max=2, Integrator_min=-2, output_min = -0.60, output_max=+0.60)
		#self.steer_for_bearing = PID.PID(P=0.02, I=0.03, D=0.02, Derivator=0, Integrator=0, Integrator_max=2, Integrator_min=-2, output_min = -0.60, output_max=+0.60)

		print self.steer_for_bearing(30)

		self.engine = None
		#self.engine.disable()

		self.navigator = None

		rospy.loginfo('sto')
		self.disable()

		rospy.loginfo('%s initialized.' % (self.name))

		if False:
			sys.exit()

		signal.signal(signal.SIGALRM, self.timeout)
	

	def X_steer_for_bearing(self, bearing):
		if bearing < -5.0:
			return(+0.15)	
		if bearing > +5.0:
			return(-0.15)	
		else:
			print 'neutral'
			return(0.0)
	
	def timeout(self, signum, frame):
		rospy.loginfo('timeout!  Disable.')
		self.disable()
		signal.alarm(0)	

	def reset_timeout(self):
		signal.alarm(self.timeout_neutral)

	def enabled(self):
		return(self._enabled)

	def enable(self):
		if self.enabled():
			rospy.loginfo('tractor already enabled')
			return(-1)

		rospy.loginfo('enabling tractor')
			
		self._enabled = True
		self.all_neutral()
		

	def all_neutral(self):
		self.transmission(0)
		self.steering(0)
		self.throttle(0)

	def disable(self):
		rospy.loginfo('disabling tractor')

		# 2016.11.12
		# This is important because otherwise an active navigator will just keep sending commands.	
		self.navigator = None	
		self.all_neutral()

		#self._enabled = False

	def set_navigator(self, navigator):
		#if str(self.navigator) == str(navigator):
		#	return

		rospy.loginfo('set navigator: %s' % (navigator))
		self.navigator = navigator

	def handle_joystick(self, joy_data):
		#rospy.loginfo('handle_joystick')
	
		joy_data_age = rospy.Time.now() - joy_data.header.stamp
		if False and abs(joy_data_age) > max_stamp_age:
			rospy.loginfo('old joy data (%d)' % (joy_data_age.secs))
			#tractor.transmission.goto_neutral()
			self.transmission(0)
			return


		self.reset_timeout()
	
		left_joy_x = joy_data.axes[0]
		left_joy_y = joy_data.axes[1]
		left_trigger = joy_data.axes[2]
		left_trigger_button = joy_data.buttons[4]
		button_back = joy_data.buttons[6]
		button_start = joy_data.buttons[7]
		button_nav = joy_data.buttons[0]

	
		#print joy_data.buttons, joy_data.axes
	
		if button_back:
			self.disable()
	
		# Sometimes the left_trigger initializes at 0.
		# Ensure it's at rest before enabling the tractor.
		if not self.enabled() and left_trigger < -0.9 and button_start:
			self.enable()
		#elif self.enabled() and left_trigger < -0.9 and button_start:
		#	self.steering.calibrate()
	

		throttle_position = (1 - left_trigger) / 2.
	
		#transmission_position = (1 - left_trigger) / 2. 
		# Reverse if left trigger.
		#if False and left_trigger_button:
		#	transmission_position = -transmission_position			

		# Go forward on left trigger button.
		if left_trigger_button:
			transmission_position = 1.0
		else:
			transmission_position = 0.0
			
		wheel_position = left_joy_x


		# If the the wheel, throttle, or transmission lever is moved, go to manual control unless the nav_set button (5) is pressed.
		if not joy_data.buttons[5]: 
			if wheel_position != 0 or transmission_position != 0 or throttle_position !=0:
				if self.navigator:
					self.set_navigator(None)

		# Otherwise, the nav_set button is pressed so take a navigation command.
		else:
			#rospy.loginfo('joy_data.buttons:' + str(joy_data.buttons))
			if False:
				None

			# Go north along line from current position.
			elif joy_data.buttons[3]:
				navigator = Line_Follower('line')
        			#navigator.load_line(
				#	navigator.frame.GeoPoint(latitude=40.88834984333333, longitude=-87.19565444333334, degrees=True),
				#	navigator.frame.GeoPoint(latitude=40.890112215, longitude=-87.19565801, degrees=True),
				#	#navigator.frame.GeoPoint(latitude=40.88853936666667, longitude=-87.195654895, degrees=True),
				#)
				navigator.load_vector(
					pointA=self.get_position(),
					azimuth=north,
				)
				self.set_navigator(navigator)

			# Go east along line from current position.
			elif joy_data.buttons[1]:
				navigator = Line_Follower(name='line')
				navigator.load_vector(
					pointA=self.get_position(),
					azimuth=(north + 90) % 360,
				)
				self.set_navigator(navigator)

			# Go south along line from current position.
			elif joy_data.buttons[0]:
				navigator = Line_Follower(name='line')
				navigator.load_vector(
					pointA=self.get_position(),
					azimuth=(north + 180) % 360,
				)
				self.set_navigator(navigator)
				
			# Go west along line from current position.
			elif joy_data.buttons[2]:
				navigator = Line_Follower(name='line')
				navigator.load_vector(
					pointA=self.get_position(),
					azimuth=(north + 270) % 360,
				)
				self.set_navigator(navigator)
				
				

			elif False and joy_data.axes[7] or joy_data.axes[6]:
				left_pad_angle = (math.degrees(math.atan2(joy_data.axes[7], joy_data.axes[6])) - 90) % 360

				self.throttle(0.5)

				self.set_navigator(Compass_Follower(left_pad_angle))


		# If we are in line following mode, handle offset changes.
		if self.navigator and self.navigator.name == 'line' and self.joy_data_prev:
			#print joy_data.axes
			if self.joy_data_prev.axes[6] == 0 and joy_data.axes[6]:
				self.navigator.offset += joy_data.axes[6] * -0.2
			if self.joy_data_prev.axes[7] == 0 and joy_data.axes[7]:
				self.navigator.throttle = max(0.0, min(1.0, self.navigator.throttle + (0.05 * joy_data.axes[7])))
				print self.navigator.throttle
			




		# Keep this for later.
		self.joy_data_prev = joy_data

		if self.navigator is not None:
			return
	
	
		# We're in manual mode so just do what the user tells us.
		self.transmission(transmission_position)
		self.steering(wheel_position)
		self.throttle(throttle_position)
	
		#rospy.loginfo('left_joy_x: %0.2f steering_position: %0.4f, transmission_position: %0.4f' % (
		#	left_joy_x, steering_position, transmission_position,
		#))
		
		return	

	def get_position(self):
		position = self.frame.GeoPoint(
			latitude=self.gps_reading['latitude'],
			longitude=self.gps_reading['longitude'],
			degrees=True,
		)
		return(position)

	
	def handle_gps_dict(self, nav_data):
		self.gps_reading = marshal.loads(nav_data.data)

		if self.navigator:
			navigation_solution = self.navigator(self.gps_reading)

			if navigation_solution:
				#print navigation_solution
				bearing = navigation_solution['bearing']
				self.steering(self.steer_for_bearing(bearing))
				self.transmission(navigation_solution['transmission'])
				self.throttle(navigation_solution['throttle'])

	def handle_nav_steering(self, steering):
		print 'steering.data:', steering.data
		self.steering(steering.data)
		self.transmission(1)

if __name__ == '__main__':
	rospy.init_node('joy_subscriber', anonymous=True)
				
	tractor = MT765(name='MT765')

	rospy.Subscriber('/joy', sensor_msgs.msg.Joy, tractor.handle_joystick)
	rospy.Subscriber('/gps/dict', std_msgs.msg.String, tractor.handle_gps_dict)
	rospy.Subscriber('/navigation/steering_pid/command', std_msgs.msg.Float64, tractor.handle_nav_steering)
	
	rospy.spin()
	sys.exit()
