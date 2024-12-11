#include <util/atomic.h>
#include <Servo.h>

// pins
#define PWM 5
#define ENC_A 2
#define ENC_B 3
#define LED 11

// consts
#define ACCEL_DIST 1.5
#define BRAKE_DIST 1.5
#define CLICKS_PER_METER 400
#define MAX_POWER 255
#define SLOW_POWER 60

float d = 0;
float totalDist = 7.0;
volatile int clicks = 0;
Servo ESC;

void setup() {
  Serial.begin(9600);
  pinMode(PWM, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, RISING);
  ESC.attach(PWM, 1100, 1900);

} 

void loop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    d = clicks / CLICKS_PER_METER;
  }
  int power = 0;
  if (d < ACCEL_DIST) {
    power = (int) (MAX_POWER * sqrt(d / ACCEL_DIST));
  } else if (d > totalDist - BRAKE_DIST) {
    power = SLOW_POWER;
  } else if (d > totalDist) {
    power = 0;
  } else {
    power = MAX_POWER;
  }
  ESC.write(power);
  if (d > totalDist) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }

}

void readEncoder() {
  int b = digitalRead(ENC_B);
  if (b > 0) {
    clicks++;
  } else {
    clicks--;
  }
}