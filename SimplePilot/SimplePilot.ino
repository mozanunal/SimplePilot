
#include <Wire.h>
#include <SPI.h>

/////////Configs////////

int rc_interval=200;
int telemetry_interval=500;

byte ID=1;

double kp_stab_yaw=0.3; double kp_stab_pitch=0.3; double kp_stab_roll=0.3; 
double kp_motor_yaw=0.2; double kp_motor_pitch=0.02; double kp_motor_roll=0.2;

double ki_stab_yaw=0; double ki_stab_pitch=0; double ki_stab_roll=0;
double ki_motor_yaw=0; double ki_motor_pitch=0; double ki_motor_roll=0;

double kd_stab_yaw=0; double kd_stab_pitch=0; double kd_stab_roll=0; 
double kd_motor_yaw=0; double kd_motor_pitch=0; double kd_motor_roll=0;

////////Variables///////

int MOTORpin1=3;
int MOTORpin2=5;
int MOTORpin3=6;
int MOTORpin4=9;



int ch_thr=0;
int ch_yaw=0;
int ch_pitch=0;
int ch_roll=0;


////////////////////////////////////////////////////////////////////////
double desired_thr=0;
double desired_yaw=0;
double desired_pitch=0;
double desired_roll=0;

double old_desired_thr=0;
double old_desired_yaw=0;
double old_desired_pitch=0;
double old_desired_roll=0;

double real_thr=0;
double real_yaw=0;
double real_pitch=0;
double real_roll=0;

double old_real_thr=0;
double old_real_yaw=0;
double old_real_pitch=0;
double old_real_roll=0;


//////////////////////////////////////////////////////////////////////

double desired_yaw_angvel=0;
double desired_pitch_angvel=0;
double desired_roll_angvel=0;

double real_yaw_angvel=0;
double real_pitch_angvel=0;
double real_roll_angvel=0;

double old_real_yaw_angvel=0;
double old_real_pitch_angvel=0;
double old_real_roll_angvel=0;

double loop_timer=5;//ms cinsinden

void setup() {
  Imu_Start();
  Motor_Arm();
  Telemetry_Start();
  Serial.begin(115200);
}

void loop() {
    unsigned long telemetry_time=millis();
    while(telemetry_time+telemetry_interval>millis())
    {
      
      Rc_Update();
      unsigned long rc_time= millis();
      
      while(rc_time+rc_interval>millis())
      {
          unsigned long loop_time=micros();
          Imu_Update();
          Motor_Update(1);
          loop_timer=(micros()-loop_time)/1000;
        
      }
      
    }
    //Telemetry_Update();
}
