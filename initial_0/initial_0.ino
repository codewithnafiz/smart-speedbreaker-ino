#include <Servo.h>
#include <NewPing.h>

#define TRIG_PIN 2
#define ECHO_PIN 3
#define SERVO_PIN 9

#define MAX_DISTANCE    20       // Maximum distance in centimeters for the speed breaker to be active
#define SPEED_THRESHOLD 3         // Minimum speed in cm/s to raise the speed breaker
#define SERVO_CLOSED_ANGLE 90     // Servo angle when speed breaker is down
#define SERVO_OPEN_ANGLE 0        // Servo angle when speed breaker is raised

NewPing sonar(TRIG_PIN, ECHO_PIN);
Servo myServo;

int previousDistance = 0;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_CLOSED_ANGLE); // Initial position, speed breaker down
}

void loop() {
  delay(50);
  
  int distance = sonar.ping_cm();
  unsigned long currentMillis = millis();
  float speed = (distance - previousDistance) / ((currentMillis - previousMillis) / 1000.0);

  if (distance > 0 && distance <= MAX_DISTANCE && speed > SPEED_THRESHOLD) {
    // If an obstacle is detected within the specified distance and speed is above threshold, raise the speed breaker
    myServo.write(SERVO_OPEN_ANGLE);
    delay(2000);
    Serial.println("Speed breaker raised");
  } else {
    // No obstacle detected or speed below threshold, keep the speed breaker down
    myServo.write(SERVO_CLOSED_ANGLE);
    Serial.println("Speed breaker down");
  }

  // Update previous values for the next iteration
  previousDistance = distance;
  previousMillis = currentMillis;
}
