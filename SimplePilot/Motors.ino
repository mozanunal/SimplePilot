/*
Motors
1   2       PIN3   PIN5
 \ /            \ /
  X              X
 /  \           / \
3    4      PIN6   PIN9


*/
#include <Servo.h>

Servo MOTOR1;
Servo MOTOR2;
Servo MOTOR3;
Servo MOTOR4;

void Motor_Arm()
{
 MOTOR1.attach(MOTORpin1);
 MOTOR2.attach(MOTORpin2);
 MOTOR3.attach(MOTORpin3);
 MOTOR4.attach(MOTORpin4);
 
 MOTOR1.writeMicroseconds(750);
 MOTOR2.writeMicroseconds(750);
 MOTOR3.writeMicroseconds(750);
 MOTOR4.writeMicroseconds(750);
 
}


  double total_error_yaw= 0; 
  double total_error_pitch= 0;
  double total_error_roll= 0;

  double old_error_yaw = 0;
  double old_error_pitch = 0;
  double old_error_roll= 0;

  double total_error_yaw_angvel= 0; 
  double total_error_pitch_angvel= 0;
  double total_error_roll_angvel= 0;

  double old_error_yaw_angvel = 0;
  double old_error_pitch_angvel = 0;
  double old_error_roll_angvel = 0;


void Motor_Update(int mode)
{
////////////////////////////////First PID/////////////////////////////////
  double error_yaw = desired_yaw-real_yaw;
  double error_pitch= desired_pitch-real_pitch;
  double error_roll=desired_roll-real_roll;
       if (desired_thr>100)//when throttle is bigger than 100//
       { 
  total_error_yaw += error_yaw*loop_timer; 
  total_error_pitch += error_pitch*loop_timer;
  total_error_roll += error_roll*loop_timer;
       }
  double error_change_yaw = (error_yaw- old_error_yaw)/loop_timer;
  double error_change_pitch = (error_pitch- old_error_pitch)/loop_timer;
  double error_change_roll = (error_roll- old_error_roll)/loop_timer;

  double desired_yaw_angvel=calculate_pid(kp_stab_yaw, ki_stab_yaw, kd_stab_yaw, error_yaw, total_error_yaw, error_change_yaw);
  double desired_pitch_angvel=calculate_pid(kp_stab_pitch, ki_stab_pitch, kd_stab_pitch, error_pitch, total_error_pitch, error_change_pitch);
  double desired_roll_angvel=calculate_pid(kp_stab_roll, ki_stab_roll, kd_stab_roll, error_roll, total_error_roll, error_change_roll);

////////////////////////////////Second PID/////////////////////////////////


  double error_yaw_angvel = desired_yaw_angvel - real_yaw_angvel;
  double error_pitch_angvel = desired_pitch_angvel - real_pitch_angvel;
  double error_roll_angvel =desired_roll_angvel - real_roll_angvel;
        if (desired_thr>100)//when throttle is bigger than 100//
       { 
  total_error_yaw_angvel += error_yaw_angvel*loop_timer; 
  total_error_pitch_angvel += error_pitch_angvel*loop_timer;
  total_error_roll_angvel += error_roll_angvel*loop_timer;
       }
  double error_change_yaw_angvel = (error_yaw_angvel- old_error_yaw_angvel)/loop_timer;
  double error_change_pitch_angvel = (error_pitch_angvel- old_error_pitch_angvel)/loop_timer;
  double error_change_roll_angvel = (error_roll_angvel- old_error_roll_angvel)/loop_timer;

  double output_motor_yaw=calculate_pid(kp_motor_yaw, ki_motor_yaw, kd_motor_yaw, error_yaw_angvel, total_error_yaw_angvel, error_change_yaw_angvel);
  double output_motor_pitch=calculate_pid(kp_motor_pitch, ki_motor_pitch, kd_motor_pitch, error_pitch_angvel, total_error_pitch_angvel, error_change_pitch_angvel);
  double output_motor_roll=calculate_pid(kp_motor_roll, ki_motor_roll, kd_motor_roll, error_roll_angvel, total_error_roll_angvel, error_change_roll_angvel);
 // Serial.print(output_motor_yaw); Serial.print("\t"); Serial.print(output_motor_pitch); Serial.print("\t");Serial.println(output_motor_roll);
////////////////////////////////Write Outputs to Motors/////////////////////////////////
      if (desired_thr>100)//when throttle is bigger than 100//
       { 
        fl_output= (desired_thr - output_motor_roll - output_motor_pitch- output_motor_yaw);
        bl_output=(desired_thr - output_motor_roll + output_motor_pitch+ output_motor_yaw);
        fr_output=(desired_thr + output_motor_roll - output_motor_pitch+ output_motor_yaw);
        br_output=(desired_thr + output_motor_roll + output_motor_pitch- output_motor_yaw);

        fl_output= setlimit(fl_output,1000,0);
        bl_output= setlimit(bl_output,1000,0);
        fr_output=setlimit(fr_output,1000,0);
        br_output=setlimit(br_output,1000,0);

        
         
        MOTOR1.writeMicroseconds(map(fl_output, 0, 1023, 700, 2000));
        MOTOR2.writeMicroseconds(map(fr_output, 0, 1023, 700, 2000));
        MOTOR3.writeMicroseconds(map(bl_output, 0, 1023, 700, 2000));
        MOTOR4.writeMicroseconds(map(br_output, 0, 1023, 700, 2000));
       }
       else
       {
        fl_output=0;
        bl_output=0;
        fr_output=0;
        br_output=0;
        MOTOR1.writeMicroseconds(750);
        MOTOR2.writeMicroseconds(750);
        MOTOR3.writeMicroseconds(750);
        MOTOR4.writeMicroseconds(750);
       }
  /////////////For next loop- ypr//////////////////     
  old_real_yaw=real_yaw;
  old_real_pitch=real_pitch;
  old_real_roll=real_roll;

  old_error_yaw = error_yaw;
  old_error_pitch = error_pitch;
  old_error_roll = error_roll;

  /////////////For next loop- ypr-angular velocity////////////////// 
  old_real_yaw_angvel=real_yaw_angvel;
  old_real_pitch_angvel=real_pitch_angvel;
  old_real_roll_angvel=real_roll_angvel;

  old_error_yaw_angvel = error_yaw_angvel;
  old_error_pitch_angvel = error_pitch_angvel;
  old_error_roll_angvel = error_roll_angvel;

}

double calculate_pid(double kp, double ki, double kd, double error, double total_error,double error_change )
{
double pid=kp*error + ki*total_error + kd*error_change ;
return pid;
}

double setlimit(double number, double maxlimit, double minlimit)
{
 if(number>maxlimit)
 {
  return maxlimit;
 }
 else if (number<minlimit)
 {
  return minlimit;
 }
 else
 {
  return number;
 }
}


