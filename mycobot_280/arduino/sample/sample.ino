#include <ros.h>
#include <std_msgs/Float32.h>
#include <Wire.h>

#include "SparkFun_VCNL4040_Arduino_Library.h" //Library: http://librarymanager/All#SparkFun_VCNL4040
VCNL4040 proximitySensor;

#include "SparkFun_LPS25HB_Arduino_Library.h"  //Library: http://librarymanager/All#SparkFun_LPS25HB
LPS25HB pressureSensor;

ros::NodeHandle  nh;

std_msgs::Float32 prox_msg;
ros::Publisher prox("prox", &prox_msg);

std_msgs::Float32 pressure_msg;
ros::Publisher pressure("pressure", &pressure_msg);

void setup()
{
  nh.initNode();
  nh.advertise(prox);
  nh.advertise(pressure);
  
  Wire.begin();
  //Wire.setClock(400000); //Increase I2C bus speed to 400kHz

  if (proximitySensor.begin() == false)
  {
    Serial.println("Proximity sensor not found. Please check wiring.");
    while (1); //Freeze!
  }
  
  pressureSensor.begin();
  if(pressureSensor.isConnected() == false)  // The library supports some different error codes such as "DISCONNECTED"
  {
    Serial.println("Pressure sensor not found. Please check wiring.");
    while (1); //Freeze!
  }
  
}

void loop()
{
  unsigned int proxValue = proximitySensor.getProximity(); 

  float pressureValue = pressureSensor.getPressure_hPa();
  float temp = pressureSensor.getTemperature_degC();
  
  prox_msg.data = proxValue;
  pressure_msg.data = pressureValue;
  
  prox.publish( &prox_msg );
  pressure.publish( &pressure_msg);
  
  nh.spinOnce();
  delay(10);
}