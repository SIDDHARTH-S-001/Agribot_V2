#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

float linear_X = 0.0;
float angular_z = 0.0;

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Implement your logic to handle cmd_vel messages here
  float linear_x = msg.linear.x;
  float angular_z = msg.angular.z;

  // Your code to control motors or perform actions based on cmd_vel values

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmdVelCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(57600);

  // Initialize any hardware, pins, or components here
}

void loop() {
  // Your main loop code goes here
  Serial.print("Linear Vel: ");
  Serial.print(linear_x);
  Serial.print("Angular Vel: ");
  Serial.print(angular_z);
  Serial.println("Linear Vel: ");

  nh.spinOnce();
}
