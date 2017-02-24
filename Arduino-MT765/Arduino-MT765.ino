/*
 * MT765 steering, transmission, and throttle control
 */

#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/UInt8.h>

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

int timeout_period = 1100;  // microseconds of no updates before reverting to passthrough, 0 to disable
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

CONTROL(throttle, 3, 11, 230, 0, 0)
CONTROL(transmission, 4, 10, 127, 0, 0)
CONTROL(steering, 5, 9, 127, 122, 132)


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
				if (timeout_period && ((long)(now - alarm_clock) >= 0)) {
					nh.logwarn("Entering passthrough mode.");

					passthrough = 1;
				}

 				// Does this device have limits?  Check them.
				else if (
					(control_measured->pt_min != 0) && (control_measured->value_in < control_measured->pt_min)
					||
					(control_measured->pt_max != 0) && (control_measured->value_in > control_measured->pt_max)
				
				) {
					// Control input limit exceeded; revert to passthrough mode.
					nh.logwarn("Input detected.  Entering passthrough mode.");
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

	// Initialize our control reading with the control at the head of the list.
	control_measured = controls;
	pciSetup(control_measured->pin_in);

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

