#!/usr/bin/python

import sys

sys.path.append('./Nvector')
sys.path.append('./pynmea2')
sys.path.append('./geographiclib-1.46.3')

import serial
import sys
import pynmea2
import string
import threading
import time
import datetime
import nvector
import numpy
import math
#import signal

import rospy
import std_msgs.msg

import marshal
import select





class GPS_NMEA():
	def __init__(
		self, 
		port, 
		baudrate=115200, 
	):
		self.port = port
		self.baudrate = baudrate
		self.uart = serial.Serial(port=self.port, baudrate=self.baudrate)
		self.streamreader = pynmea2.NMEAStreamReader(self.uart)
		#self.thread = threading.Thread(target=self.consumer)

		self.prev_heading = 0.0

		self.clear_position()

		self.frame = nvector.FrameE(name='WGS84')

		self.update_time = time.time()

		# It's not running, but it hasn't been started either.
		self.stopped = False

		# restart
		if False:
			#self.send_command('$PNVGRST,W')
			self.send_command('$PNVGRST,F')
			sys.exit()

		if True:
			#send_command(receiver_nmea, '$PNVGRZB,PNVGBLS,1,GGA,5,PNVGIMU,1,RMC,1,PNVGBSS,1')

			#self.send_command('$PNVGVER')
			#self.send_command('$PNVGRZB')

			#self.send_command('$PNVGRZB,PNVGBLS,1,GGA,5,PNVGIMU,1,RMC,1,PNVGBSS,1,VTG,1,HDT,1')
			self.send_command('$PNVGRZB,PNVGBLS,1,GGA,5,PNVGIMU,1,RMC,1,PNVGBSS,1,VTG,1,HDT,1')
			#self.send_command('$PNVGRZB,PNVGBLS,1,PNVGIMU,1,RMC,1,PNVGBSS,1,VTG,1,HDT,1')
			#self.send_command('$PNVGRZB,RMC,1,HDT,1')
			#self.send_command('$PNVGRZB,PNVGPSS,1')
			#self.send_command('$PNVGRTK,MODE,4,PVTRATE,10,SYSGPS,1,SYSGLO,1,SFMODE,3')

	def start(self):
		self.uart.flushOutput()
		self.thread.start()

	def stop(self):
		self.stopped = True

	def clear_position(self):
		self.timestamp = None
		self.latitude = None
		self.longitude = None
		self.heading = None
		self.mode_indicator = None
		self.correction_age = None
		self.course_over_ground = None
		self.speed_over_ground = None
		self.accel_north = None
		self.accel_east = None

	def clear_position_if_new_timestamp(self, timestamp):
		if not timestamp == self.timestamp:
			self.clear_position()
			self.timestamp = timestamp

	def N_degrees_to_E_radians(self, n_degrees):
		return(numpy.deg2rad((90 - n_degrees) % 360))

	def nvg_timestamp(self, s):
		return(datetime.time(
		 	hour=int(s[0:2]),
			minute=int(s[2:4]),
			second=int(s[4:6]),
			microsecond=int(s[7:]) * 10000,
		))

	#def consumer(self):

	def timeout_handler(self, signum, frame):
		print '*************** timeout **************'
		self.stopped = True
		#raise Exception, 'timeout'

	def get_update(self, timeout=None):
		self.stopped = False	
		#if timeout:
		#	signal.signal(signal.SIGALRM, self.timeout_handler)
		#	signal.alarm(timeout)

		# Consume everything in the buffer.
		# I do this instead of a simple flush() in case I'm in the midst of receiving a sentence.
		# I'd rather wait for the next complete batch.
	        #while (self.uart.inWaiting() > 0):
		#	self.uart.readline()

		if True:
			#print 'consume_sentences(port=%s)' % (self.port)

			while not self.stopped:
				#if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				#	self.stopped = True

				message = self.get_message()
				#print self.stopped, message
				print '->', message

				message_type = type(message)
			
				if message_type == pynmea2.types.talker.RMC:
					if not message.is_valid:
						print 'discard invalid message'
						continue

					#$GPRMC,155401.30,A,4053.1927345,N,08711.6866736,W,0.00,0.00,130317,0.0,E,D*2C


					timestamp = message.timestamp
					self.clear_position_if_new_timestamp(timestamp)

					self.mode_indicator = message.data[11]

					#print 'mode:', self.mode_indicator

					if not self.mode_indicator in (u'R', u'F', u'D'):
						print 'mode_indicator:', self.mode_indicator, self.correction_age
						continue

					self.latitude = message.latitude
					self.longitude = message.longitude
					#print 'latitude/longitude: %f/%f' % (self.latitude, self.longitude)

					#self.speed_over_ground = message.speed_over_ground * 1.852 # knots to Km/hour
					#self.course_over_ground = message.course_over_ground
					self.speed_over_ground = float(message.data[6]) * 1.852 # knots to Km/hour
					self.course_over_ground = float(message.data[7])
					#self.course_over_ground = 123
				
					#print self.speed_over_ground

				elif message_type == pynmea2.nmea.ProprietarySentence:
					message_id = message.data[0]

					#[u'BLS', u'172600.30', u'-0.563', u'-0.029', u'-0.003', u'0.563', u'182.93', u'-0.33', u'R']
					if True and message_id == u'BLS':
						if len(message.data) < 9:
							print 'bad message:', message.data
							continue

						mode_indicator = message.data[8]
						#print '2mode_indicator:', mode_indicator, self.correction_age
						if not mode_indicator in (u'R', u'F'):
							print '1mode_indicator:', mode_indicator, self.correction_age
							continue

						timestamp = self.nvg_timestamp(message.data[1])
						self.clear_position_if_new_timestamp(timestamp)

						#self.heading = self.N_degrees_to_E_radians(float(message.data[6]))
						try:
							self.heading = float(message.data[6])
							print 'BLS heading:', self.heading

						except:
							print 'bad BLS'
							continue
							raise Exception, 'Can not parse heading from "%s".' % (message.data[6])

						#print 'heading:', float(message.data[6])

					# $PNVGBSS,3526,0,3526,7863.169,0.4*61
					# no timestamp
					elif message_id == u'BSS':
						#print 'BSS', message.data
						correction_age_s = message.data[5]

						if correction_age_s:
							self.correction_age = float(correction_age_s)
							#print 'correction_age:', self.correction_age

					elif message_id == u'IMU':
						#print message.data[5:7]
						self.accel_north = float(message.data[5])
						self.accel_east = float(message.data[6])
						#print math.atan2(accel_north, accel_east)
						#print '%+2.3f %+2.3f' % (accel_north, accel_east)
					elif False:
						print message


				# This doesn't work.  Might be pynmea-1.
				elif True and message_type == pynmea2.types.talker.VTG:
					#print 'VTG', message.data
					#sys.exit()
					self.course_over_ground = float(message.data[0])
					self.speed_over_ground = float(message.data[6]) * 1000

				elif message_type == pynmea2.types.talker.HDT:
					#self.heading = self.N_degrees_to_E_radians(float(message.data[0]))
					heading_s = message.data[0]
					if heading_s:
						self.heading = float(heading_s)
						print 'HDT heading:', self.heading
				elif False:
					print message_type

				# I wish there was a better way to know that we have all of the sentences for this pulse.
				#if self.latitude and self.longitude and self.heading and self.course_over_ground and self.speed_over_ground: 


				#if self.uart.inWaiting() == 0 and self.latitude and self.longitude and self.heading:
				#print self.latitude, self.longitude, self.heading

				if (self.latitude is not None) and (self.longitude is not None) and (self.heading is not None) and (self.correction_age is not None):
				#if (self.latitude is not None) and (self.longitude is not None) and (self.correction_age is not None):
					# kludge!
					#if self.heading is None:
					#	self.heading = self.prev_heading
					#else:
					#	self.prev_heading = self.heading

					#print 'update!'
					now = time.time()
					period = now - self.update_time

					if period > 1.0:
						raise Exception, 'period: %0.2f' % (period)

					update = {
						#'position': self.frame.GeoPoint(
						#	latitude=self.latitude,	
						#	longitude=self.longitude,
						#	degrees=True,
						#),
						'latitude': self.latitude,
						'longitude': self.longitude,
						'heading': (self.heading + 180) % 360,
						#'heading': self.heading,
						'course_over_ground': self.course_over_ground,
						#'course_over_ground': self.heading,
						'speed_over_ground': self.speed_over_ground,	
						'mode_indicator': self.mode_indicator,
						#'accel_north': self.accel_north,
						#'accel_east': self.accel_east,
						#'timestamp': self.timestamp,
						'timestamp': str(self.timestamp),
						'period': period,
						'correction_age': self.correction_age,
					}
					self.update_time = now

					
					self.clear_position()
					#signal.alarm(0)
					return(update)

					if self.navigation_callback:
						self.navigation_callback(
							position=self.frame.GeoPoint(
								latitude=self.latitude,	
								longitude=self.longitude,
								degrees=True,
							),
							heading=self.heading,
							#course_over_ground=self.course_over_ground,
							course_over_ground=self.heading,
							speed_over_ground=self.speed_over_ground,
						)

					# So we don't call it again.
					self.clear_position()

					sys.exit()


		#except:
		else:
			print '*******'
			return(None)

	def send_command(self, command):
		command_string = str(pynmea2.parse(command))
		print 'command_string:', command_string
		self.uart.write('\r\n\r\n')
		self.uart.write(command_string)
		self.uart.write('\r\n')


	def get_message(self):
		sentence = None
		while (sentence is None):
			try:
				sentence = self.streamreader.next()
				#print ':', sentence
			except:
				print 'sentence exception'
				continue

		if len(sentence) != 1:
			raise Exception, 'sentence: %s' % (str(sentence))

		return(sentence[0])

if __name__ == '__main__':
	print 'checkpoint 1'
	gps_nmea = GPS_NMEA(port='/dev/gps_nmea')
	print 'checkpoint 2'

	rospy.init_node('GPS')
	print 'checkpoint 3'
	pub = rospy.Publisher('/gps/dict', std_msgs.msg.String, queue_size=1, tcp_nodelay=False)
	print 'checkpoint 4'

	#while (not sys.stdin in select.select([sys.stdin], [], [], 0)[0]):
	while (True):
		#print 'checkpoint 5'
		update = gps_nmea.get_update()
		print 'update:', update
		pub.publish(std_msgs.msg.String(marshal.dumps(update)))

	print 'exiting'
	time.sleep(10)
