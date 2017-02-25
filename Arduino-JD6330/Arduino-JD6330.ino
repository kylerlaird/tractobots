/*
	JD6330 

  	transmission (shuttle)
		linear actuator
		PID
		motor driver
		neutral sense input

	steering
		proportional hydraulic valves
		PID
		2xMOSFET

	throttle
		analog?  PWM?
		PID with RPM or ground speed?

	3-pt.
		up
		down
		direct? MOSFET?

	engine enable
		MOSFET

	starter
		MOSFET

	lights
		?xMOSFET

	horn
		MOSFET
	
	PTO
		MOSFET
		3-wire?

	RPM?


	Arduino (8 digital out, 6 PWM out, 8 analog in)

	input
		steer position
		shuttle actuator position
		shuttle neutral sensor
		steering wheel override
		shuttle selector position
		seat switch?
		RPM?

	dual motor driver.0
		shuttle fore (PWM)
		shuttle aft (PWM)
		powershift fore (PWM)
		powershift aft (PWM)
	
	MOSFET4.0
		steer left (PWM)
		steer right (PWM)
		3-pt down
		3-pt up

	MOSFET4.1
		horn
		running lights
		work lights

	MOSFET4.2
		engine enable
		starter
		PTO



 */

#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include <PID_v1.h>

#include <ros.h>
// #include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

unsigned long timeout_period = 1500;    // milliseconds of no updates before stopping
unsigned long alarm_clock;

char message[100];

int loop_count = 0;

int steering_left_pin = 3;
int steering_right_pin = 5;
int steering_position_pin = 1;
double steering_position = 128 << 2 ;
double steering_desired = 128 << 2;
double steering_correction = 0;
// PID steering_PID(&steering_position, &steering_correction, &steering_desired, 0.1, 0.02, 0.01, DIRECT);
PID steering_PID(&steering_position, &steering_correction, &steering_desired, 0.6, 0.02, 0.01, DIRECT);

int shuttle_aft_pin = 6;
int shuttle_fore_pin = 9;
int shuttle_position_pin = 0;
double shuttle_position = 128 << 2;
double shuttle_desired = 128 << 2;
double shuttle_correction = 0;
PID shuttle_PID(&shuttle_position, &shuttle_correction, &shuttle_desired, 5, 1, 0, DIRECT);

int shuttle_speed = 0;
int steering_speed = 0;

int ignition_pin = 4;

int hitch_down_pin = 7;
int hitch_up_pin = 8;

std_msgs::UInt8 steering_position_message;
ros::Publisher steering_pub("/steering/get", &steering_position_message);
ros::Subscriber<std_msgs::UInt8> steering_sub("/steering/put", set_steering);

std_msgs::UInt8 shuttle_position_message;
ros::Publisher shuttle_pub("/transmission/shuttle/get", &shuttle_position_message);
ros::Subscriber<std_msgs::UInt8> shuttle_sub("/transmission/shuttle/put", set_shuttle);

std_msgs::Bool ignition_enable_message;
ros::Publisher ignition_pub("/ignition/get", &ignition_enable_message);
ros::Subscriber<std_msgs::Bool> ignition_sub("/ignition/put", set_ignition);

std_msgs::UInt8 hitch_message;
ros::Publisher hitch_pub("/hitch/get", &hitch_message);
ros::Subscriber<std_msgs::UInt8> hitch_sub("/hitch/put", set_hitch);


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

void ignition(byte i) {
	digitalWrite(ignition_pin, i);
	ignition_enable_message.data = i;

	activity(); 
}

void set_ignition( const std_msgs::Bool& cmd_msg) {
	nh.logwarn("set_ignition()");

	ignition(cmd_msg.data);
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

void hitch(int i) {
	if (i == 0) {
		digitalWrite(hitch_down_pin, 0);
		digitalWrite(hitch_up_pin, 0);
	} else if (i == 1) {
		digitalWrite(hitch_up_pin, 0);
		digitalWrite(hitch_down_pin, 1);
	} else if (i == 2) {
		digitalWrite(hitch_down_pin, 0);
		digitalWrite(hitch_up_pin, 1);
	}
	hitch_message.data = i;
}

void set_hitch( const std_msgs::UInt8& cmd_msg) {
	nh.logwarn("set_hitch()");

	hitch(cmd_msg.data);
	activity();
}


void setup() {
	// Disable the ignition while we get started.
	pinMode(ignition_pin, OUTPUT);
	ignition(0);

	// Configure hitch.
	pinMode(hitch_down_pin, OUTPUT);
	pinMode(hitch_up_pin, OUTPUT);
	hitch(0);		
	
	// Initialize our rosserial link.
	nh.initNode();

	nh.advertise(ignition_pub);

	nh.advertise(steering_pub);
	nh.subscribe(steering_sub);

	nh.advertise(shuttle_pub);
	nh.subscribe(shuttle_sub);

	nh.advertise(hitch_pub);
	nh.subscribe(hitch_sub);

	//steering_PID.SetOutputLimits(-255.0, +255.0);
	//steering_PID.SetOutputLimits(-25.0, +25.0);
	steering_PID.SetOutputLimits(-20.0, +20.0);
	steering_PID.SetSampleTime(50);
	steering_PID.SetMode(AUTOMATIC);

	shuttle_PID.SetOutputLimits(-255.0, +255.0);
	shuttle_PID.SetSampleTime(50);
	shuttle_PID.SetMode(AUTOMATIC);

	// Enable the ignition.
	ignition(1);
	nh.subscribe(ignition_sub);
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

		ignition_pub.publish( &ignition_enable_message );
		hitch_pub.publish( &hitch_message );
	}


	delay(1);
}























/* 
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);


void setup()
{
  //initialize the variables we're linked to
    Input = analogRead(0);
      Setpoint = 100;

        //turn the PID on
	  myPID.SetMode(AUTOMATIC);
	  }

	  void loop()
	  {
	    Input = analogRead(0);
	      myPID.Compute();
	        analogWrite(3,Output);
		}

ros::NodeHandle nh;
char buf[100];

typedef struct control_s {
        char *name;
        char pin_in;
	char pin_out;
	byte pt_min;
	byte pt_max;
	byte value_in;
	byte value_out;
	struct control_s *next;
} control_t;

volatile char passthrough = 1;

int timeout_period = 1100000;  // microseconds of no updates before going to neutral

volatile unsigned long alarm_clock;

void snooze() {
	alarm_clock = millis() + timeout_period;
}


#define CONTROL(n, pi, po, s, ptmin, ptmax) \
	static control_t control_ ## n = { \
		name:(char *)#n, \
		pin_in:pi, \
		pin_out:po, \
		pt_min:ptmin, \
		pt_max:ptmax, \
	}; \
	void put_ ## n( const std_msgs::UInt8& msg ) { \
		control_ ## n.value_out = msg.data; \
		snooze(); \
	}; \
	void get_ ## n( ) { \
	}; \


#define INIT_CONTROL(n) \
	control_ ## n.next=controls; \
	controls=&control_ ## n; \
	static ros::Subscriber<std_msgs::UInt8> n ## _sub("/" #n "/put", &put_ ## n ); \
	nh.subscribe(n ## _sub); \
	pinMode(control_ ## n.pin_out, OUTPUT); \


// This is our linked list of controls.
control_t *controls = NULL;

CONTROL(shuttle, 3, 11, 230, 0, 0)
CONTROL(transmission, 4, 10, 127, 0, 0)
CONTROL(steering, 5, 9, 127, 120, 138)


int enable_switch = 12;
byte enable_switch_position = 1;

// This is the control we're currently measuring.
// Start at the head of the list.
volatile control_t *control_measured;
volatile unsigned long rise_time = 0;
volatile unsigned long fall_time = 0;

#define PASSTHROUGH_THRESHOLD 100

// pin change interrupt
ISR (PCINT2_vect) {
	unsigned long now = micros();

	byte new_state = digitalRead(control_measured->pin_in);

	// rise_time < fall_time < now

	// rising
	if ( new_state ) {
		// Do we have a valid read?  No rollover.  Not uninitialized.
		if (rise_time != 0 && fall_time != 0 && now > fall_time && fall_time > rise_time) {
			control_measured->value_in = (byte)(256 * (fall_time - rise_time) / (now - rise_time));

			// If we're out of passthrough mode...
			if (passthrough == 0) {
				// Has there been activity recently?
				if ((long)(now - alarm_clock) >= 0) {
					passthrough = 1;
				}

 				// Does this device have limits?  Check them.
				else if (
					(control_measured->pt_min != 0) && (control_measured->value_in < control_measured->pt_min)
					||
					(control_measured->pt_max != 0) && (control_measured->value_in > control_measured->pt_max)
				
				) {
					// Control input limit exceeded; revert to passthrough mode.
					passthrough = 1;
				}
			}


			// If in passthrough mode, set the output pin to the value just now read from the input pin.	
			if (passthrough) {
				analogWrite(control_measured->pin_out, control_measured->value_in);
			} else {
				analogWrite(control_measured->pin_out, control_measured->value_out);
			}

			// Disable interrupts for all pins (in this group).
			// ***could be more efficient
    			*digitalPinToPCMSK(control_measured->pin_in) = 0;

			// Move to next control;
			//control_measured = (control_measured->next == NULL ? controls : control_measured->next);

			if (control_measured->next == NULL) {
				// We're at the end of the list.  Return to the head.
				control_measured = controls;
			} else {
				control_measured = control_measured->next;
			}

			// Initialize times for next control.
			rise_time = 0;
			fall_time = 0;

			// Enable interrupts for pin on next control.
			// ***could be more efficient
    			*digitalPinToPCMSK(control_measured->pin_in) |= bit (digitalPinToPCMSKbit(control_measured->pin_in)); 
    			PCIFR  |= bit (digitalPinToPCICRbit(control_measured->pin_in)); // clear any outstanding interrupt

		// This is the initial rise.
		} else {
			rise_time = now;
			fall_time = 0;
		}

	// falling
	} else {
		fall_time = now;
	}

}

void pciSetup(byte pin) 
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}


void setup()
{

	nh.initNode();

	// Initialize each control.
	INIT_CONTROL(throttle);
	INIT_CONTROL(transmission);
	INIT_CONTROL(steering);

	// Initialize enable switch input.
	pinMode(enable_switch, INPUT_PULLUP);

	
}

void loop()
{
	nh.spinOnce();
	if (1) {
		enable_switch_position = digitalRead(enable_switch);
		// sprintf(buf, "loop: %u/%u", digitalRead(enable_switch), passthrough); nh.logwarn(buf);

		// Switch out of passthrough mode.
		if ((passthrough > 0) && (enable_switch_position == 0)) {
			nh.logwarn("Leaving passthrough mode.");

			// Stop passing values from input pins to output pins.
			passthrough = 0;
			
			// Set the output pins to our desired values.	
			for (control_t *control=controls; control; control = control->next) {
				sprintf(buf, "%s (%u) = %d", control->name, control->pin_out, control->value_out); nh.logwarn(buf);
				analogWrite(control->pin_out, control->value_out);
			}
		}


		if (0) {
			for (control_t *control=controls; control; control = control->next) {
				sprintf (buf, "%s: %u->%u, %u/%u",
					control->name,
					control->pin_in, control->pin_out,
					control->value_in, control->value_out
				);
				nh.logwarn(buf);

			}
		}
	}
	delay(1);
}

*/

