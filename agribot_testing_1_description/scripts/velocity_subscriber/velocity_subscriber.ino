#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String pwmMsg;
ros::Publisher rightPWM("/right_pwm", &pwmMsg);
ros::Publisher leftPWM("/left_pwm", &pwmMsg);

void rightWheelVelCallback(const std_msgs::Float64& msg)
{
  float rightWheelVel = msg.data;
  Serial.print("Right Wheel Velocity: ");
  Serial.println(rightWheelVel);

  // Convert PWM value to string and publish
  String tempMsg = "RightPWM:" + String(int(rightWheelVel));
  tempMsg.toCharArray(pwmMsg.data, sizeof(pwmMsg.data));
  rightPWM.publish(&pwmMsg);
}

void leftWheelVelCallback(const std_msgs::Float64& msg)
{
  float leftWheelVel = msg.data;
  Serial.print("Left Wheel Velocity: ");
  Serial.println(leftWheelVel);

  // Convert PWM value to string and publish
  String tempMsg = "LeftPWM:" + String(int(leftWheelVel));
  tempMsg.toCharArray(pwmMsg.data, sizeof(pwmMsg.data));
  leftPWM.publish(&pwmMsg);
}

void setup()
{
  Serial.begin(57600);

  // Set up ROS
  nh.initNode();

  // Advertise publishers
  nh.advertise(rightPWM);
  nh.advertise(leftPWM);

  // Create subscribers
  ros::Subscriber<std_msgs::Float64> rightWheelSub("/right_wheel_vel", &rightWheelVelCallback);
  ros::Subscriber<std_msgs::Float64> leftWheelSub("/left_wheel_vel", &leftWheelVelCallback);

  // Register subscribers
  nh.subscribe(rightWheelSub);
  nh.subscribe(leftWheelSub);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
