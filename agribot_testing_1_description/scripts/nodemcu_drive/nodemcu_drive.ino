#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char *ssid = "Micromax-HS2";
const char *password = "Micromax";

AsyncWebServer server(80);

#define pwm_1 D1
#define dir_1 D2
#define pwm_2 D5
#define dir_2 D6

float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;
int left_pwm = 0;
int right_pwm = 0;
String left_dir = "LOW";
String right_dir = "HIGH";

void setup() {
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");  
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Route for setting wheel speed
  server.on("/set_speed", HTTP_GET, [](AsyncWebServerRequest *request){
      if (request->hasParam("leftwheel") && request->hasParam("leftspeed") && request->hasParam("rightwheel") && request->hasParam("rightspeed")) {
        String leftwheel = request->getParam("leftwheel")->value();
        String leftspeedStr = request->getParam("leftspeed")->value();
        String rightswheel = request->getParam("rightwheel")->value();
        String rightspeedStr = request->getParam("rightspeed")->value();

        leftWheelSpeed = leftspeedStr.toFloat();
        rightWheelSpeed = rightspeedStr.toFloat();
  
//        if (wheel == "right") {
//          rightWheelSpeed = speedStr.toFloat();
//        } else if (wheel == "left") {
//          leftWheelSpeed = speedStr.toFloat();
//        }
  
        request->send(200, "text/plain", "Speed set successfully");
      } else {
        request->send(400, "text/plain", "Missing parameters");
      }
  });


  // Start server
  server.begin();
}

void loop() {

  if(leftWheelSpeed >= 0 ){
    digitalWrite(dir_1, HIGH);   
    left_dir = "HIGH";
  }

  else if(leftWheelSpeed < 0){
    digitalWrite(dir_1, LOW); 
    left_dir = "LOW";
  }

  else if(rightWheelSpeed >= 0 ){
    digitalWrite(dir_2, LOW);   
    right_dir = "HIGH";
  }

  else if(rightWheelSpeed < 0){
    digitalWrite(dir_2, HIGH); 
    right_dir = "LOW";
  }

  // For now not giving full PWM
  left_pwm = int((abs(leftWheelSpeed)/1.75)*255);
  right_pwm = int((abs(rightWheelSpeed)/1.75)*255);

  if (left_pwm > 255){
    left_pwm = 255;
  }

  if (right_pwm > 255){
    right_pwm = 255;
  }

  analogWrite(pwm_1, left_pwm);
  analogWrite(pwm_2, right_pwm);

  Serial.print("Left Wheel PWM: ");
  Serial.print(left_pwm);
  Serial.print(" Left Wheel Direction: ");
  Serial.print(left_dir);
  Serial.print(" Right Wheel PWM: ");
  Serial.print(right_pwm);
  Serial.print(" Right Wheel Direction: ");
  Serial.println(right_dir);

  delay(100);  // Adjust delay based on your needs
}
