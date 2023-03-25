/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-force-sensor
 */
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Wire.h>

#define FORCE_SENSOR_PIN_FORCE A0 // the FSR and 10K pulldown are connected to A0
#define FORCE_SENSOR_PIN_TOUCH A1 // the FSR and 10K pulldown are connected to A1

ros::NodeHandle nh;

std_msgs::Float32 force_msg;
ros::Publisher force("force", &force_msg);

std_msgs::Float32 touch_msg;
ros::Publisher touch("touch", &touch_msg);


void setup() {
     nh.initNode();
     nh.advertise(force);
     nh.advertise(touch);

     Wire.begin();
     
  // Serial.begin(9600);
  // Serial.print("force");
  // Serial.print(",");
  // Serial.println("touch");
}

void loop() {
  unsigned int analogReadingForce = analogRead(FORCE_SENSOR_PIN_FORCE);
  unsigned int analogReadingTouch = analogRead(FORCE_SENSOR_PIN_TOUCH);
  unsigned int forceValue = analogReadingForce;
  unsigned int touchValue = analogReadingTouch;
  
  force_msg.data = forceValue;
  touch_msg.data = touchValue;

  force.publish(&force_msg);
  touch.publish(&touch_msg);

  nh.spinOnce();
  

  // Serial.print(analogReadingForce); // print the raw analog reading
  
  // Serial.print(",");
  
  // Serial.println(analogReadingTouch); // print the raw analog reading

 
  delay(10);
}
