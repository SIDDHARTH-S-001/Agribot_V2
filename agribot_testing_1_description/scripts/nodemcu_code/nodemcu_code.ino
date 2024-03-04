#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char *ssid = "Micromax-HS2";
const char *password = "Micromax";

AsyncWebServer server(80);

#define pwm_1 16
#define dir_1 17
#define pwm_2 18
#define dir_2 19

float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;
uint8_t left_dir = HIGH;
uint8_t right_dir = HIGH;

void setup() {
  Serial.begin(115200);

  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);

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
  server.on("/set_speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("wheel") && request->hasParam("speed")) {
      String wheel = request->getParam("wheel")->value();
      String speedStr = request->getParam("speed")->value();

      float speed = speedStr.toFloat();

      // Determine direction based on speed sign
      uint8_t direction = (speed > 0) ? HIGH : LOW;

      // Map the absolute value of speed to PWM value
      int pwmValue = map(abs(speed), 0, 0.35, 0, 255);

      if (wheel == "right") {
        rightWheelSpeed = speed;
        right_dir = direction;
        digitalWrite(dir_1, direction);
        analogWrite(pwm_1, pwmValue);
      } 
      
      else if (wheel == "left") {
        leftWheelSpeed = speed;
        left_dir = direction;
        digitalWrite(dir_2, direction);
        analogWrite(pwm_2, pwmValue);
      }

      request->send(200, "text/plain", "Speed set successfully. Direction: " + String(direction) + ", PWM Value: " + String(pwmValue));
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  // Start server
  server.begin();
}

void loop() {
  Serial.print("Left Wheel Speed: ");
  Serial.print(leftWheelSpeed);
  Serial.print(" Left Wheel Direction: ");
  Serial.print(left_dir);
  Serial.print(" Right Wheel Speed: ");
  Serial.print(rightWheelSpeed);
  Serial.print(" Left Wheel Direction: ");
  Serial.println(right_dir);

  delay(100);  // Reduce delay to 0.1 sec
}
