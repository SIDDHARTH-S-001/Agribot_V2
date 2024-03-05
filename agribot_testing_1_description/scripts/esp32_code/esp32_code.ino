#include <WiFi.h>
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
  server.on("/set_speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("wheel") && request->hasParam("speed")) {
      String wheel = request->getParam("wheel")->value();
      String speedStr = request->getParam("speed")->value();

      if (wheel == "right") {
        rightWheelSpeed = speedStr.toFloat();
      } else if (wheel == "left") {
        leftWheelSpeed = speedStr.toFloat();
      }

      request->send(200, "text/plain", "Speed set successfully");
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });

  // Start server
  server.begin();
}

void loop() {
  Serial.print("Left Wheel Speed: ");
  Serial.println(leftWheelSpeed);
  Serial.print("Right Wheel Speed: ");
  Serial.println(rightWheelSpeed);

  if (leftWheelSpeed >= 0) {
    digitalWrite(dir_1, HIGH);
    left_dir = "HIGH";
  } else if (leftWheelSpeed < 0) {
    digitalWrite(dir_1, LOW);
    left_dir = "LOW";
  }

  if (rightWheelSpeed >= 0) {
    digitalWrite(dir_2, HIGH);
    right_dir = "HIGH";
  } else if (rightWheelSpeed < 0) {
    digitalWrite(dir_2, LOW);
    right_dir = "LOW";
  }

  left_pwm = int((abs(leftWheelSpeed) / 0.35) * 255);
  right_pwm = int((abs(rightWheelSpeed) / 0.35) * 255);

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
