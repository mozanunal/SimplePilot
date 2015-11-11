/*
In this section you must chose or add the sensor you use.
At least 1 IMU sensor must be chosen
The Imu, which is chosen, must have 2 fuction(Imu_Start and Imu_Update)   
*/


///////////MPU6050---IMU//////////
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>
#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"

int raw_values[9];
float ypr[3]; 
float val[9];
FreeIMU my3IMU = FreeIMU();

void Imu_Start()
{
  Wire.begin();
  delay(5);
  my3IMU.init(); // the parameter enable or disable fast mode
  delay(5);
}
void Imu_Update()
{
  my3IMU.getYawPitchRoll(ypr);
  real_yaw=ypr[0];
  real_pitch=ypr[1];
  real_roll=ypr[2];
  
  real_yaw_angvel=(real_yaw-old_real_yaw)/loop_timer;
  real_pitch_angvel=(real_pitch-old_real_pitch)/loop_timer;
  real_roll_angvel=(real_roll-old_real_roll)/loop_timer;


}
//////////////////////////////

