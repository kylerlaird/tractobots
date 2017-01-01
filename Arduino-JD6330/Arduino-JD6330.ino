#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include <PID_v1.h>

#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

unsigned long timeout_period = 1500;    // milliseconds of no updates before stopping
unsigned long alarm_clock;

char message[100];

int loop_count = 0;

int steering_left_pin = 3;
int steering_right_pin = 5;
int steering_position_pin = 3;
double steering_position = 128 << 2 ;
double steering_desired = 128 << 2;
double steering_correction = 0;
PID steering_PID(&steering_position, &steering_correction, &steering_desired, 0.1, 0.02, 0.01, DIRECT);

int shuttle_aft_pin = 6;
int shuttle_fore_pin = 9;
int shuttle_position_pin = 0;
double shuttle_position = 128 << 2;
double shuttle_desired = 128 << 2;
double shuttle_correction = 0;
PID shuttle_PID(&shuttle_position, &shuttle_correction, &shuttle_desired, 5, 1, 0, DIRECT);

int shuttle_speed = 0;
int steering_speed = 0;

void snooze() {
	// nh.logwarn("snooze()");
        
	alarm_clock = millis() + timeout_period;
}

void activity() {
	// nh.logwarn("activity()");

	digitalWrite(13, HIGH-digitalRead(13)); //toggle led
	snooze();
}


void set_steering( const std_msgs::UInt8& cmd_msg) {
	 nh.logwarn("set_steering()");

	steering_desired = cmd_msg.data << 2;
	activity(); 
}

void set_shuttle( const std_msgs::UInt8& cmd_msg) {
	nh.logwarn("set_shuttle()");

	shuttle_desired = cmd_msg.data << 2;
	activity(); 
}

void shuttle_correct() {
	shuttle_speed = abs(shuttle_correction);

	if (shuttle_speed < 100) {
		analogWrite(shuttle_aft_pin, 0);			
		analogWrite(shuttle_fore_pin, 0);
	} else if (shuttle_correction < 0) {
		// fore correction
		analogWrite(shuttle_aft_pin, 0);			
		analogWrite(shuttle_fore_pin, shuttle_speed);
	} else if (shuttle_correction > 0) {
		// aft correction
		analogWrite(shuttle_fore_pin, 0);
		analogWrite(shuttle_aft_pin, shuttle_speed);			
	}
}

void steering_correct() {
	steering_speed = abs(steering_correction);
	// steering_speed = 235;
	// steering_speed = 255;
	
	if (steering_speed < 2) {
		analogWrite(steering_left_pin, 0);			
		analogWrite(steering_right_pin, 0);
	} else if (steering_correction > 0) {
		analogWrite(steering_left_pin, 0);			
		analogWrite(steering_right_pin, 230 + steering_speed);
	} else if (steering_correction < 0) {
		// aft correction
		analogWrite(steering_right_pin, 0);
		analogWrite(steering_left_pin, 230 + steering_speed);			
	}
}


std_msgs::UInt8 steering_position_message;
ros::Publisher steering_pub("/steering/current", &steering_position_message);
ros::Subscriber<std_msgs::UInt8> steering_sub("/steering/setting", set_steering);

std_msgs::UInt8 shuttle_position_message;
ros::Publisher shuttle_pub("/transmission/shuttle/current", &shuttle_position_message);
ros::Subscriber<std_msgs::UInt8> shuttle_sub("/transmission/shuttle/setting", set_shuttle);

void setup() {
	// Initialize our rosserial link.
	nh.initNode();

	nh.advertise(steering_pub);
	nh.subscribe(steering_sub);

	nh.advertise(shuttle_pub);
	nh.subscribe(shuttle_sub);

	//steering_PID.SetOutputLimits(-255.0, +255.0);
	//steering_PID.SetOutputLimits(-25.0, +25.0);
	steering_PID.SetOutputLimits(-20.0, +20.0);
	steering_PID.SetSampleTime(50);
	steering_PID.SetMode(AUTOMATIC);

	shuttle_PID.SetOutputLimits(-255.0, +255.0);
	shuttle_PID.SetSampleTime(50);
	shuttle_PID.SetMode(AUTOMATIC);

}

void loop() {
	// nh.logwarn("loop()");
  
	steering_position = analogRead(steering_position_pin);
	steering_PID.Compute();
	steering_correct();

	shuttle_position = analogRead(shuttle_position_pin);
	shuttle_PID.Compute();
	shuttle_correct();


	// Do ROS stuff.
	nh.spinOnce();

	loop_count += 1;
	if (loop_count % 1000 == 0) {
		if (0) {
			sprintf(
				message, "%03d/%03d/%03d %03d/%03d/%03d", 
				int(steering_position) >> 2, int(steering_desired) >> 2, int(steering_correction),
				int(shuttle_position) >> 2, int(shuttle_desired) >> 2, int(shuttle_correction)
			);
		}

		dtostrf(shuttle_position, 6, 2, message);
		sprintf(message + strlen(message), "/");

		dtostrf(shuttle_desired, 6, 2, message + strlen(message));
		sprintf(message + strlen(message), "/");

		dtostrf(shuttle_correction, 6, 2, message + strlen(message));

		sprintf(message + strlen(message), "/%03d - ", shuttle_speed);




		dtostrf(steering_position, 6, 2, message + strlen(message));
		sprintf(message + strlen(message), "/");

		dtostrf(steering_desired, 6, 2, message + strlen(message));
		sprintf(message + strlen(message), "/");

		dtostrf(steering_correction, 6, 2, message + strlen(message));

		sprintf(message + strlen(message), "/%03d", steering_speed);



		nh.logwarn(message);

		steering_position_message.data = int(steering_position) >> 2;
		steering_pub.publish( &steering_position_message );

		shuttle_position_message.data = int(shuttle_position) >> 2;
		shuttle_pub.publish( &shuttle_position_message );
	}


	delay(1);
}
