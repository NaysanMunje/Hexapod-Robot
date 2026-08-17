#include <ESP32Servo.h>

Servo servo;
const int SERVO_PIN = 11;

// Center position; adjust if your horn zero is different
const int CENTER = 90;

void setup() {
  Serial.begin(115200);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(CENTER);
  delay(1000);
  Serial.println("Servo test starting");
}

void loop() {
  // +45 degrees
  servo.write(CENTER + 45);
  Serial.println("Moved +45");
  delay(3000);

  // -45 degrees (back the other way, returns to center)
  servo.write(CENTER - 45);
  Serial.println("Moved -45");
  delay(3000);

  // Optional: return to center before next cycle
  servo.write(CENTER);
  delay(1000);
}
