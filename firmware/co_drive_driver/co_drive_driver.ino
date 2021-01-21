/*
 * Co-Drive Hardware Driver
 *
 * Controls pneumatic system for Co-Drive
 *
 * Subscribes to
 * /codrive_driver/gripper_state    (0 = Off, 1 = On)
 * /codrive_driver/relay_state      (0 = Neutral, 1 = Close, 2 = Open)
 * both are UInt8s, relay must be turned on for states to function
 *
 */

#include <ros.h>
#include <std_msgs/UInt8.h>

//Gripper States
#define GRIP_NEUTRAL 0
#define GRIP_CLOSE 1
#define GRIP_OPEN 2

//Relay States
#define RELAY_OFF 0
#define RELAY_ON 1


//Control Lines
#define OUT_A 2 //Close gripper solenoid
#define OUT_B 3 //Open gripper solenoid
#define OUT_C 4 //Turns air pump on


//+3.3V Vref
#define VREF A0


//Co-Drive PSI
#define PSI A1


//Presure Limits in 0.1 PSI
#define MAX 100
#define MIN -50


//Relay Bitmask
#define RELAY_MASK B00011111


//Gripper Bitmask
#define GRIP_MASK B00011100

//=====================================================================================================================

byte relay_state = 0; //default to relay off
byte gripper_state = 0; //default to neutral grip
byte prev_gripper_state = 0;  //default to neutral grip prev. state


//ROS Arduino reference: http://wiki.ros.org/rosserial_arduino/

ros::NodeHandle nh;

//Gripper Subscriber
void gripperCb (const std_msgs::UInt8& msg){

  gripper_state = msg.data;

}

ros::Subscriber<std_msgs::UInt8> grip_sub("codrive_driver/grip_state", &gripperCb);


//Relay Subscriber
void relayCb (const std_msgs::UInt8& msg){

  relay_state = msg.data;

}

ros::Subscriber<std_msgs::UInt8> relay_sub("codrive_driver/relay_state", &relayCb);


byte getPSI () {

  float comp = 3.3 / analogRead(VREF);
  return (byte) (7.288016*analogRead(PSI)*comp - 21.65923);

}


//TODO: NEED TO CHECK IF ALL THESE FUNCTIONS ARE LOGICAL FOR THE ACTUAL SYSTEM

//======================================================================================================

void relaxGrip() {

  if (((getPSI() > 3) || (getPSI() < -3)) && (prev_gripper_state == GRIP_CLOSE)){

      PORTD = PORTD | B00010100;  //Write Pin 2 and 4 High -> FETs pulled Low
      PORTD = PORTD & B11110111;  //Clear Pin 3 -> FET pulled High

  } else if (((getPSI() > 3) || (getPSI() < -3)) && (prev_gripper_state == GRIP_OPEN)) {

      PORTD = PORTD | B00001100;  //Write Pin 2 and 3 High -> Fets pulled Low
      PORTD = PORTD & B11101111;  //Clear Pin 4 -> FET pulled High

  } else {

      PORTD = PORTD | B00011100;    //Write pins 2-4 high -> FETs pulled Low
      prev_gripper_state = gripper_state;

  }
}


void closeGrip() {

  if(getPSI() < MAX) {

      PORTD = PORTD | B00001000;  //Write Pin 3 High -> Fet pulled Low
      PORTD = PORTD & B11101011;  //Clear Pin 2 and 4 -> FETs pulled High

  } else {

      PORTD = PORTD | B00011100;    //Write pins 2-4 high -> FETs pulled Low
      prev_gripper_state = gripper_state;

  }
}


void openGrip() {

  if(getPSI() > MIN) {

      PORTD = PORTD | B00010000;    //Write Pin 4 High -> FET pulled Low
      PORTD = PORTD & B11110011;    //Clear Pin 2 and 3 -> FETs pulled High

  } else {

      PORTD = PORTD | B00011100;    //Write pins 2-4 high -> FETs pulled Low
      prev_gripper_state = gripper_state;

  }

}


void relayOn() {

  // This code utilizes a bitmask to flip pins #-# at the same exact time to ensure that they are not creating any
  // short. This provides the neccassary 80 mA +/- 10% to properly drive the onboard relay.
  PORTB = PORTB | RELAY_MASK;
  
}


void relayOff() {

  //Disable Relay
  PORTB = PORTB & (~RELAY_MASK);

}


void setup() {

  
    //+24V Digital Line Control via FETs
    pinMode(OUT_A, OUTPUT);
    pinMode(OUT_B, OUTPUT);
    pinMode(OUT_C, OUTPUT);


    //Pin 5 High Impedance - This ensures the output bank will function properly
    pinMode(5, INPUT);
    //The reason for this was due to a physical error in the boards design related
    //To how much current digital pins are capable of sourcing.
    //Do not alter this unless the physical issue has been repaired or you are
    //Implementing an alternate fix

  

    //Port B Output Pins - Used to provide 100 mA to the relay.
    pinMode(8,OUTPUT);
    pinMode(9,OUTPUT);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
    //This requires pin 8-12 to be jumped to pin 5, and for
    //Pin 5 to be high impedance (ie: input). The other option
    //is to cut the pin 5 trace and attach to pin 5 of K1 relay
    //ENSURE THESE ARE ONLY USED VIA PORT MANIPULATION    !!!
    //IF THESE PINS AREN'T THE SAME YOU'RE SHORTING THEM  !!!


    //ROS Node Handle Initialization
    nh.initNode();
    nh.subscribe(grip_sub);
    nh.subscribe(grip_sub);

}

void loop() {

  switch (relay_state){

    case RELAY_OFF:
      relayOff();
      nh.logdebug("Relay State: RELAY_OFF");
      break;

    case RELAY_ON:
      relayOn();
      nh.logdebug("Relay State: RELAY_ON");
      break;
  }

  switch (gripper_state){

    case GRIP_NEUTRAL:
      relaxGrip();
      nh.logdebug("State: GRIP_NEUTRAL");
      break;

    case GRIP_CLOSE:
      closeGrip();
      nh.logdebug("State: GRIP_CLOSE");
      break;

    case GRIP_OPEN:
      openGrip();
      nh.logdebug("State: GRIP_OPEN");
      break;
  }

  nh.spinOnce();

}
