#include <util/atomic.h>
#include <Servo.h>

// pins
#define PWM 9
#define ENC_A 2  //white
#define ENC_B 3  // green
#define LED 13

// consts
#define ACCEL_DIST 0.5
#define BRAKE_DIST 1.5
#define CLICKS_PER_METER 3480.0
#define MAX_POWER 180
#define MIN_ACCEL_POWER 60
#define SLOW_POWER 30

float d = 0;
float prevD = 0;
float totalDist = 3.0;
volatile int counter = 0;
Servo ESC;
float t = 0;
long prevT = 0;
float deltaT;
float power = 0;

void setup() {
  pinMode(2, INPUT_PULLUP);  // internal pullup input pin 2

  pinMode(3, INPUT_PULLUP);  // internalเป็น pullup input pin 3

  Serial.begin(9600);
  pinMode(PWM, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_A, INPUT_PULLUP);  // internal pullup input pin 2

  pinMode(ENC_B, INPUT_PULLUP);  // internalเป็น pullup input pin 3
                                 //Setting tp interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);

  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
  ESC.attach(PWM, 1000, 2000);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    d = -counter / CLICKS_PER_METER;
  }
  float deltaD = d - prevD;
  prevD = max(d, 0.0001);

  long currT = micros();
  deltaT = ((float)(currT - prevT)) / (1000000.0);
  prevT = currT;
  t += deltaT;
  if (t > 5) {
    if (d < ACCEL_DIST) {
      power = max(MAX_POWER * sqrt(d / ACCEL_DIST), MIN_ACCEL_POWER);
      digitalWrite(LED_BUILTIN, LOW);
    } else if ((d > totalDist - BRAKE_DIST) && (d < totalDist)) {
      power = MAX_POWER-MAX_POWER * sqrt(sqrt(sqrt((d-(totalDist-BRAKE_DIST))/BRAKE_DIST)));
      digitalWrite(LED_BUILTIN, LOW);
    } else if (d > totalDist) {
      power = 0;
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      power = MAX_POWER;
      digitalWrite(LED_BUILTIN, LOW);
    }
    power = max(min(power, MAX_POWER), 0);
    Serial.print(power);
    Serial.print(" ");
    Serial.println(d);
  }
  ESC.write((int) power);
}


void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}