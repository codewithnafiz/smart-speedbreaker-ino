#include <Servo.h>
#include <NewPing.h>

#define TRIG_PIN 2
#define ECHO_PIN 3
#define SERVO_PIN 9
#define BUZZER_PIN 10  // Add this line for the buzzer

#define MAX_DISTANCE 20
#define SPEED_THRESHOLD 4
#define SERVO_CLOSED_ANGLE 90
#define SERVO_OPEN_ANGLE 0

NewPing sonar(TRIG_PIN, ECHO_PIN);
Servo myServo;

int previousDistance = 0;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  pinMode(BUZZER_PIN, OUTPUT);  // Initialize the buzzer pin
  myServo.write(SERVO_CLOSED_ANGLE);
}

void loop() {
  delay(50);
  
  int distance = sonar.ping_cm();
  unsigned long currentMillis = millis();
  float speed = (distance - previousDistance) / ((currentMillis - previousMillis) / 1000.0);

  if (distance > 0 && distance <= MAX_DISTANCE && speed > SPEED_THRESHOLD) {
    // If an obstacle is detected within the specified distance and speed is above threshold, raise the speed breaker
    myServo.write(SERVO_OPEN_ANGLE);
    digitalWrite(BUZZER_PIN, HIGH);  // Activate the buzzer
    Serial.println("Speed breaker raised");
    delay (2000);
    
  } else {
    // No obstacle detected or speed below threshold, keep the speed breaker down
    myServo.write(SERVO_CLOSED_ANGLE);
    digitalWrite(BUZZER_PIN, LOW);   // Deactivate the buzzer
    Serial.println("Speed breaker down");
  }

  previousDistance = distance;
  previousMillis = currentMillis;
}
