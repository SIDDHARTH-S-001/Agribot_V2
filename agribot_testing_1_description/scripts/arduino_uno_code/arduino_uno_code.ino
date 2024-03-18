#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#define Motor_L_Dir 4
#define Motor_L_Stp 5
#define Motor_R_Dir 7
#define Motor_R_Stp 6

ros::NodeHandle nh;

float linear_X = 0.0;
float angular_z = 0.0;
float sepration = 0.48;

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Implement your logic to handle cmd_vel messages here
  float linear_x = msg.linear.x;
  float angular_z = msg.angular.z;

  // Your code to control motors or perform actions based on cmd_vel values
  int left_pwm = (linear_x - angular_z*(sepration)/2)*450;
  int right_pwm = (linear_x + angular_z*(sepration)/2)*450;

  if(left_pwm > 255){
    left_pwm = 255;
  }

  if(right_pwm > 255){
    right_pwm = 255;
  }

  if(left_pwm < -255){
    left_pwm = -255;
  }

  if(right_pwm < -255){
    right_pwm = -255;
  }

  if(right_pwm > 0){
    digitalWrite(Motor_R_Dir, 1);
    analogWrite(Motor_R_Stp, right_pwm);
  }
  
  else{
    digitalWrite(Motor_R_Dir, 0);
    analogWrite(Motor_R_Stp, abs(right_pwm));
  }
  
  if(left_pwm > 0){
    digitalWrite(Motor_L_Dir, 1);
    analogWrite(Motor_L_Stp, left_pwm);
  }
  
  else{
    digitalWrite(Motor_L_Dir, 0);
    analogWrite(Motor_L_Stp, abs(left_pwm));
  }

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmdVelCallback);  

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  pinMode(Motor_L_Dir,OUTPUT);
  pinMode(Motor_L_Stp,OUTPUT); 
  pinMode(Motor_R_Dir,OUTPUT);
  pinMode(Motor_R_Stp,OUTPUT); 
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();
}
