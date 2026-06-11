#include <Arduino.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>


#define STEP1_PIN D5
#define DIR1_PIN  D2
#define EN1_PIN   D4

#define STEP2_PIN D6
#define DIR2_PIN  D3
#define EN2_PIN   D9

#define MODE_SWITCH_PIN  GPIO_NUM_42
#define MAGNET_PIN      GPIO_NUM_13


#define SERVO_PIN D10
#define PUMP_PIN D7

#define SPEED_HZ     5000
#define ACCELERATION 4000
#define OBSTACLE_CM  16

#define SERVO_MIN 30
#define SERVO_MAX 150
#define SERVO_DELAY 30

#define PUMP_ACTIVE_MS 1000
#define PUMP_DESACTIVE_MS 500

// ==========================
// ESP-NOW
// ==========================
uint8_t receiverMac[] = {0xE8, 0x06, 0x90, 0xA0, 0x48, 0x38};

uint8_t message; // 0 = jaune, 1 = bleu

// ==========================
// TIMING
// ==========================
unsigned long lastSend = 0;
#define SEND_INTERVAL 50

bool magnetTriggered = false;
FastAccelStepperEngine engine;
FastAccelStepper *motorLeft;
FastAccelStepper *motorRight;

Servo headServo;

TaskHandle_t motorTaskHandle = NULL;

volatile bool pauseMotors = false;

long targetLeft = 0;
long targetRight = 0;

enum PathMode {
  PATH_YELLOW,
  PATH_BLUE
};

PathMode selectedPath;

void updateServo() {
  static int pos = (SERVO_MIN + SERVO_MAX) / 2;
  static int dir = 1;
  static unsigned long t = 0;

  if (millis() - t < SERVO_DELAY) return;
  t = millis();

  pos += dir;
  if (pos >= SERVO_MAX) { pos = SERVO_MAX; dir = -5; }
  if (pos <= SERVO_MIN) { pos = SERVO_MIN; dir = 5; }

  headServo.write(pos);
}

void setServoPosition(int position) {
  if (position >= SERVO_MIN && position <= SERVO_MAX) {
    headServo.write(position);
    delay(SERVO_DELAY);  // wait for servo to reach position
  }
}

void activatePump(unsigned long duration) {
  digitalWrite(PUMP_PIN, HIGH);
  delay(duration);
  digitalWrite(PUMP_PIN, LOW);
}

void waitMotors() {
  while (motorLeft->isRunning() || motorRight->isRunning()) {

    while (pauseMotors) {
      vTaskDelay(pdMS_TO_TICKS(20));  // pause propre
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void safeMove(long leftDelta, long rightDelta) {

  targetLeft  = motorLeft->getCurrentPosition()  + leftDelta;
  targetRight = motorRight->getCurrentPosition() + rightDelta;

  motorLeft->moveTo(targetLeft);
  motorRight->moveTo(targetRight);

  while (motorLeft->isRunning() || motorRight->isRunning()) {

    if (pauseMotors) {
      motorLeft->stopMove();
      motorRight->stopMove();

      while (pauseMotors) {
        vTaskDelay(pdMS_TO_TICKS(20));
      }

      // reprise EXACTE
      motorLeft->moveTo(targetLeft);
      motorRight->moveTo(targetRight);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void runYellowPath() {
  safeMove(1300, -1300);   // avancer
  safeMove(-700, -700); 
  safeMove(3400, -3400);
  activatePump(PUMP_ACTIVE_MS);  // active la pompe via MOSFET
  setServoPosition(90);    // activate servo at 90 degrees  
  setServoPosition(0);
  safeMove(-3400, 3400);
  safeMove(700, 700); // tourner G
  safeMove(3300, -3300);
  safeMove(-700, -700);
  safeMove(-1500,1500);
  safeMove(2900, -2900);
  safeMove(-2900, 2900);
  safeMove(700, 700);
  safeMove(2000, -2000);
  safeMove(-700, -700);
  safeMove(-1500,1500);
  safeMove(3300, -3300);

  setServoPosition(90); 
  activatePump(PUMP_DESACTIVE_MS);
  setServoPosition(0);
 


}

void runBluePath() {
   safeMove(1300, -1300);   // avancer
   activatePump(PUMP_ACTIVE_MS);  // active la pompe via MOSFET
  safeMove(700, 700); 
 safeMove(3200, -3200);
  setServoPosition(90);    // activate servo at 90 degrees
  setServoPosition(0);     // activate servo at 0 degrees
  safeMove(-3200, 3200);
  safeMove(-700, -700); // tourner G
  safeMove(3300, -3300);
  safeMove(700, 700);
 safeMove(-1500,1500);
  safeMove(3200, -3200);
  safeMove(-3200, 3200);
  safeMove(-700, -700);
  safeMove(2000, -2000);
  safeMove(700, 700);
  safeMove(-1500,1500);
  safeMove(3300, -3300);

  setServoPosition(90); 
  activatePump(PUMP_DESACTIVE_MS);
  setServoPosition(0);
}

void motorTask(void *param) {

  while (true) {

    if (selectedPath == PATH_YELLOW) {
      runYellowPath();
    } else {
      runBluePath();
    }

    Serial.println("🏁 Trajectoire terminée");
    vTaskSuspend(NULL);   // arrêt définitif
  }
}



void setup() {

  Serial.begin(115200);

  delay(1000); // attendre que le moniteur série soit prêt

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MAGNET_PIN, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);


  engine.init();

  motorLeft = engine.stepperConnectToPin(STEP1_PIN);
  motorRight = engine.stepperConnectToPin(STEP2_PIN);

  motorLeft->setDirectionPin(DIR1_PIN);
  motorLeft->setEnablePin(EN1_PIN);
  motorLeft->setAutoEnable(true);
  motorLeft->setSpeedInHz(SPEED_HZ);
  motorLeft->setAcceleration(ACCELERATION);

  motorRight->setDirectionPin(DIR2_PIN);
  motorRight->setEnablePin(EN2_PIN);
  motorRight->setAutoEnable(true);
  motorRight->setSpeedInHz(SPEED_HZ);
  motorRight->setAcceleration(ACCELERATION);

  headServo.attach(SERVO_PIN, 500, 2400);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println(" ESP-NOW FAIL");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(" Erreur ajout peer");
    return;
  }

  Serial.println(" Prêt - attente aimant...");

}

void loop() {

  // Lecture capteur magnétique
  bool magnetState = (digitalRead(MAGNET_PIN) == LOW);

  // Détection front (important)
  if (magnetState && !magnetTriggered) {
    magnetTriggered = true;

    Serial.println(" AIMANT DETECTÉ → START ENVOI");

    // Lecture couleur UNE SEULE FOIS
    if (digitalRead(MODE_SWITCH_PIN) == HIGH) {
      message = 0;
      selectedPath = PATH_YELLOW;
      Serial.println("JAUNE");
    } else {
      message = 1;
      selectedPath = PATH_BLUE;
      Serial.println("BLEU");
    }

    // Création des tâches FreeRTOS après détection
    xTaskCreatePinnedToCore(
      motorTask,
      "Motor Task",
      4096,
      NULL,
      2,
      &motorTaskHandle,
      1   // Core 1
    );
  }

  // Envoi seulement si aimant actif
  if (magnetTriggered && millis() - lastSend > SEND_INTERVAL) {
    lastSend = millis();

    esp_err_t result = esp_now_send(receiverMac, &message, sizeof(message));

    if (result == ESP_OK) {
      Serial.print(" Envoyé : ");
      Serial.println(message);
    } else {
      Serial.println(" Erreur envoi");
    }
    updateServo();
  }

  // Debug capteur (important)
  Serial.print("Magnet: ");
  Serial.println(digitalRead(MAGNET_PIN));

  delay(25);
}