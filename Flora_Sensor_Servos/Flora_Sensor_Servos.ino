// THE ROBOTIC FLOWER
// Code put together by Inga Woods-Waight
// TikTok - @ingawoods.waight
// Instagram - @ingawoods.waight


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// constants won't change
const int TRIG_PIN = 6;              // Arduino pin connected to Ultrasonic Sensor's TRIG pin
const int ECHO_PIN = 7;              // Arduino pin connected to Ultrasonic Sensor's ECHO pin
const int DISTANCE_THRESHOLD = 150;  // centimeters

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

// variables will change:
float duration_us, distance_cm;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);  // set arduino pin to output mode
  pinMode(ECHO_PIN, INPUT);   // set arduino pin to input mode
  Serial.println("4 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void loop() {
  int pos = 125;
  int pos_m = 485;

  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);
  // calculate the distance
  distance_cm = 0.017 * duration_us;

  if (distance_cm < DISTANCE_THRESHOLD)
    close();  // rotate servo motor to 0 degree
  else
    open();  // rotate servo motor to 180 degree

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // delay(500);
}

void open() {
  int pos = 125;
  for (int pos = 125; pos < 485; pos += 10)
    ;
  {  // Loop with movement slow from 30 degrees to 150 degrees
    pwm.setPWM(0, 0, pos);
    pwm.setPWM(1, 0, pos);
    pwm.setPWM(2, 0, pos);
    pwm.setPWM(3, 0, pos);
    Serial.println("0");
    delay(50);
  }

  delay(5000);
}

void close() {
  int pos_m = 485;
  for (int pos_m = 485; pos_m > 125; pos_m -= 10)
    ;
  {  // Loop with movement slow from 150 degrees to 30 degrees
    pwm.setPWM(0, 0, pos_m);
    pwm.setPWM(1, 0, pos_m);
    pwm.setPWM(2, 0, pos_m);
    pwm.setPWM(3, 0, pos_m);
    Serial.println("180");
    delay(50);
  }

  delay(5000);
}
