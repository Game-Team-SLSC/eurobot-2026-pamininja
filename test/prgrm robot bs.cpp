#include <Arduino.h>
#include <Servo.h>
#include <TM1637Display.h>

#define SERVO1_PIN 3
#define SERVO2_PIN 4
#define SERVO3_PIN 5
#define SERVO4_PIN 6
#define SERVO5_PIN 12
#define POT_PIN A0

#define DISPLAY_CLK_PIN 2
#define DISPLAY_DIO_PIN 13

#define BTN1_PIN 7
#define BTN2_PIN 8
#define BTN3_PIN 9
#define BTN4_PIN 10
#define BTN5_PIN 11

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
TM1637Display display(DISPLAY_CLK_PIN, DISPLAY_DIO_PIN);

void setup() {
  Serial.begin(9600);
  display.setBrightness(0x0f);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(BTN5_PIN, INPUT_PULLUP);

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);
}

void loop() {
  servo1.write(digitalRead(BTN1_PIN) == LOW ? 90 : 0);
  servo2.write(digitalRead(BTN2_PIN) == LOW ? 90 : 0);
  servo3.write(digitalRead(BTN3_PIN) == LOW ? 90 : 0);
  servo4.write(digitalRead(BTN4_PIN) == LOW ? 90 : 0);
  servo5.write(digitalRead(BTN5_PIN) == LOW ? 90 : 0);

  int rawValue = analogRead(POT_PIN);
  int displayValue = map(rawValue, 0, 1023, 0, 200);
  displayValue = constrain(displayValue, 0, 200);

  display.showNumberDec(displayValue, false, 4, 0);

  Serial.print("Potentiometre: ");
  Serial.println(displayValue);

  delay(200);
}
