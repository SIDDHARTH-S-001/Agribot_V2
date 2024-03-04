//v#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Float64.h>

//const char *ssid = "Micromax-HS2";
//const char *password = "Micromax";
//const char *host = "192.168.143.136";
//const int port = 11311;  // Change to your ROS master port

//WiFiClient espClient;
ros::NodeHandle nh;

int rightMotorPinPWM = 16;  // Change to the actual PWM pin for the right motor
int rightMotorPinDir = 17;  // Change to the actual direction pin for the right motor
int leftMotorPinPWM = 18;   // Change to the actual PWM pin for the left motor
int leftMotorPinDir = 19;   // Change to the actual direction pin for the left motor

void rightWheelVelCallback(const std_msgs::Float64 &msg)
{
  float vel = msg.data;

  // Set the direction based on the sign of velocity
  if (vel >= 0)
  {
    digitalWrite(rightMotorPinDir, HIGH);  // Forward
  }
  else
  {
    digitalWrite(rightMotorPinDir, LOW);  // Reverse
  }

  // Map the magnitude of velocity to motor speed (20-255)
  int speed = map(constrain(abs(vel), 0, 0.35), 0, 0.35, 20, 255);

  analogWrite(rightMotorPinPWM, speed);
}

void leftWheelVelCallback(const std_msgs::Float64 &msg)
{
  float vel = msg.data;

  // Set the direction based on the sign of velocity
  if (vel >= 0)
  {
    digitalWrite(leftMotorPinDir, HIGH);  // Forward
  }
  else
  {
    digitalWrite(leftMotorPinDir, LOW);  // Reverse
  }

  // Map the magnitude of velocity to motor speed (20-255)
  int speed = map(constrain(abs(vel), 0, 0.35), 0, 0.35, 20, 255);

  analogWrite(leftMotorPinPWM, speed);
}

void setup()
{
  Serial.begin(115200);

  // Connect to Wi-Fi
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED)
//  {
//    delay(1000);
//    Serial.println("Connecting to WiFi...");
//  }
//  Serial.println("Connected to WiFi");

  // Set up ROS
//  nh.getHardware()->setConnection(espClient);
//  nh.initNode(host, port);
  nh.initNode();
  nh.subscribe("/right_wheel_vel", &rightWheelVelCallback);
  nh.subscribe("/left_wheel_vel", &leftWheelVelCallback);

  // Set motor pins as outputs
  pinMode(rightMotorPinPWM, OUTPUT);
  pinMode(rightMotorPinDir, OUTPUT);
  pinMode(leftMotorPinPWM, OUTPUT);
  pinMode(leftMotorPinDir, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
