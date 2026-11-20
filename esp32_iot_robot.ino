
//UserName: mine_rescue
//Password:Projectiot@123
/*
 * =========================================================================================
 * Project      : Autonomous & IoT Cloud Robot with 16x2 LCD Telemetry & Critical Alert
 * File         : esp32_iot_robot.ino
 * Target Board : ESP32 DevKit V1
 * Directory    : G:\1. 2026-2027\1. Project Work\3. Durga\15. Camreea robot\Hardware_Code\esp32_iot_robot\
 * =========================================================================================
 * 
 * AUTO CAMERA LINK ON IOT CLOUD DASHBOARD & 16x2 LCD:
 *   - Automatically detects the ESP32-CAM's IP address via UDP wireless packets.
 *   - Displays the camera link directly on the Arduino IoT Cloud `istatus` widget:
 *       "CAM: http://192.168.43.52 | T:28C H:65% | ..."
 *     (You can copy-paste it directly to Chrome on your Laptop / PC!)
 *   - Shows the camera IP on Line 1 of the 16x2 LCD! Zero Serial Port opening needed.
 * =========================================================================================
 */

#include "thingProperties.h"
#include <WiFiUdp.h>
#include <DHT.h>
#include <LiquidCrystal.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
// Exact 16x2 LCD Pins requested by user
const int LCD_RS = 2;
const int LCD_EN = 15;
const int LCD_D4 = 19;
const int LCD_D5 = 18;
const int LCD_D6 = 5;
const int LCD_D7 = 4;

// Motor Driver Pins (Zero LCD overlap)
#define MOTOR_IN1       25
#define MOTOR_IN2       26
#define MOTOR_IN3       27
#define MOTOR_IN4       14

#define DHTPIN          23
#define DHTTYPE         DHT11

#define GAS_PIN         34   // ADC1 Channel 6
#define FLAME_PIN       35   // Digital Flame Input
#define BUZZER_PIN      13   // Alert Buzzer

// ==========================================
// 2. HARDWARE OBJECTS
// ==========================================
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
const uint16_t UDP_PORT = 4210;
bool udpStarted = false;

// Auto-detected Camera Web Link
String camWebLink = "Connecting...";
String camIpOnly  = "WAITING...";

// Non-blocking Timing Intervals
unsigned long lastSensorReadMs   = 0;
const unsigned long SENSOR_INTERVAL_MS = 2000;

unsigned long lastPersonPacketMs = 0;
const unsigned long PERSON_TIMEOUT_MS = 3000;

unsigned long lastLcdUpdateMs    = 0;
const unsigned long LCD_UPDATE_MS = 400;

// Sensor Readings & Classification States
float currentTemp = 28.0;
float currentHum  = 65.0;
int currentGas    = 450;
char tempStatus   = 'L';
char humStatus    = 'N';
char gasStatus    = 'N';
bool isCriticalHazard = false;

String robotStatusText = "IDLE";

// ==========================================
// 3. AUTO SQUARE PATROL STATE MACHINE
// ==========================================
bool autoSquareActive = false;
int squareStep = 0;
unsigned long stepStartTime = 0;
unsigned long currentStepDuration = 0;

const unsigned long MOVE_FORWARD_TIME = 1500;
const unsigned long TURN_RIGHT_TIME   = 650;

// ==========================================
// 4. MOTOR CONTROL PRIMITIVES
// ==========================================
void stopRobot() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  robotStatusText = "STOPPED";
}

void forward() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  robotStatusText = "FORWARD";
}

void backward() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  robotStatusText = "BACKWARD";
}

void turnLeft() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  robotStatusText = "LEFT TURN";
}

void turnRight() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  robotStatusText = "RIGHT TURN";
}

void startSquarePatrol() {
  autoSquareActive = true;
  squareStep = 0;
  stepStartTime = millis();
  forward();
  currentStepDuration = MOVE_FORWARD_TIME;
  robotStatusText = "SQUARE S1/4";
  Serial.println("[AUTO] Square Patrol Started: Moving Forward (Side 1/4)");
}

void updateSquarePatrol() {
  if (!autoSquareActive) return;

  unsigned long now = millis();
  if (now - stepStartTime >= currentStepDuration) {
    stepStartTime = now;
    squareStep++;

    if (squareStep >= 8) {
      autoSquareActive = false;
      stopRobot();
      icontrol = "S";
      robotStatusText = "SQUARE DONE";
      Serial.println("[AUTO] Square Patrol Completed! Robot Stopped.");
      return;
    }

    if (squareStep % 2 == 0) {
      forward();
      currentStepDuration = MOVE_FORWARD_TIME;
      int sideNumber = (squareStep / 2) + 1;
      robotStatusText = "SQUARE S" + String(sideNumber) + "/4";
      Serial.printf("[AUTO] Moving Forward (Side %d/4)\n", sideNumber);
    } else {
      turnRight();
      currentStepDuration = TURN_RIGHT_TIME;
      int turnNumber = (squareStep / 2) + 1;
      robotStatusText = "SQUARE R" + String(turnNumber) + "/4";
      Serial.printf("[AUTO] Turning Right (Corner %d/4)\n", turnNumber);
    }
  }
}

// ==========================================
// 5. 16x2 LCD DISPLAY MANAGER (<= 16 Chars)
// ==========================================
void updateLcdScreen() {
  char line0[17];
  char line1[17];

  // LINE 0 FORMAT: "T:28C:L  H:65%:N" (Strictly 16 Characters)
  int tVal = (isnan(currentTemp)) ? 0 : (int)currentTemp;
  int hVal = (isnan(currentHum))  ? 0 : (int)currentHum;
  snprintf(line0, sizeof(line0), "T:%02dC:%c  H:%02d%%:%c", tVal, tempStatus, hVal, humStatus);

  // Line 1: Cycles between Telemetry & Camera IP Address
  static unsigned long lastCycleMs = 0;
  static int displayCycle = 0;
  if (millis() - lastCycleMs > 2500) {
    displayCycle = (displayCycle + 1) % 2;
    lastCycleMs = millis();
  }

  // If Critical Hazard is Active, Override Line 1 with Alarm
  if (isCriticalHazard) {
    if (iflame) {
      snprintf(line1, sizeof(line1), "*CRITICAL: FIRE*");
    } else if (gasStatus == 'H') {
      snprintf(line1, sizeof(line1), "*CRITICAL: GAS!*");
    } else if (tempStatus == 'H') {
      snprintf(line1, sizeof(line1), "*CRITICAL: TEMP*");
    }
  } else {
    if (displayCycle == 0) {
      // Screen A: Gas + Flame (FD/FN) + Person (P:YES / P:NO )
      const char* flameStr  = iflame  ? "FD" : "FN";
      const char* personStr = iperson ? "P:YES" : "P:NO ";
      snprintf(line1, sizeof(line1), "G:%04d%c %s %s", currentGas, gasStatus, flameStr, personStr);
    } else {
      // Screen B: Shows Camera IP on LCD! e.g. "CAM:192.168.43.52"
      snprintf(line1, sizeof(line1), "CAM:%-12s", camIpOnly.c_str());
    }
  }

  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

// ==========================================
// 6. SETUP FUNCTION
// ==========================================
void setup() {
#if defined(RTC_CNTL_BROWN_OUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#elif defined(RTC_CNTL_BROWNOUT_REG)
  WRITE_PERI_REG(RTC_CNTL_BROWNOUT_REG, 0);
#endif

  Serial.begin(115200);
  delay(1000);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  stopRobot();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(GAS_PIN, INPUT);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AI ROBOT NODE   ");
  lcd.setCursor(0, 1);
  lcd.print("CONNECTING IOT..");

  dht.begin();

  istatus  = "Connecting IoT...";
  icontrol = "S";
  igas     = 0;
  iflame   = false;
  iperson  = false;

  Serial.println("\n=======================================================");
  Serial.println("  ESP32 IOT CLOUD ROBOT WITH 16x2 LCD & SENSORS START  ");
  Serial.println("=======================================================");

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  Serial.println("[SETUP] Waiting for Wi-Fi and Cloud Connection...");
}

// ==========================================
// 7. MAIN LOOP (NON-BLOCKING)
// ==========================================
void loop() {
  ArduinoCloud.update();

  unsigned long currentMs = millis();

  // 1. UDP RECEIVER & AUTO CAMERA IP DISCOVERY
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      if (udp.begin(UDP_PORT)) {
        udpStarted = true;
        Serial.printf("[UDP] Listening for Camera on Port %d\n", UDP_PORT);
      }
    } else {
      int packetSize = udp.parsePacket();
      if (packetSize > 0) {
        char packetBuffer[64];
        int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
        if (len > 0) packetBuffer[len] = '\0';
        while (udp.available()) udp.read();

        String msg = String(packetBuffer);
        msg.trim();

        // Automatically extract Camera IP address from the incoming UDP packet!
        camIpOnly = udp.remoteIP().toString();
        camWebLink = "http://" + camIpOnly;

        if (msg == "1" || msg.indexOf("PERSON") >= 0) {
          iperson = true;
          lastPersonPacketMs = currentMs;
          Serial.printf("[CAMERA ALERT] Person Spotted! Cam Link: %s\n", camWebLink.c_str());
        }
      }
    }
  } else {
    udpStarted = false;
  }

  // Auto-reset Person presence after 3 seconds of silence
  if (iperson && (currentMs - lastPersonPacketMs > PERSON_TIMEOUT_MS)) {
    iperson = false;
  }

  // 2. Non-blocking Auto Square Patrol State Machine
  updateSquarePatrol();

  // 3. Sensor Reading, Classification & Critical Alert Check (Every 2 seconds)
  if (currentMs - lastSensorReadMs >= SENSOR_INTERVAL_MS) {
    lastSensorReadMs = currentMs;

    currentGas = analogRead(GAS_PIN);
    igas = currentGas;

    if (currentGas < 1000) gasStatus = 'L';
    else if (currentGas <= 2200) gasStatus = 'N';
    else gasStatus = 'H';

    iflame = (digitalRead(FLAME_PIN) == LOW);

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t)) {
      currentTemp = t;
      if (currentTemp < 30.0) tempStatus = 'L';
      else if (currentTemp <= 36.0) tempStatus = 'N';
      else tempStatus = 'H';
    }

    if (!isnan(h)) {
      currentHum = h;
      if (currentHum < 35.0) humStatus = 'L';
      else if (currentHum <= 70.0) humStatus = 'N';
      else humStatus = 'H';
    }

    if (iflame || gasStatus == 'H' || tempStatus == 'H') {
      isCriticalHazard = true;
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      isCriticalHazard = false;
      digitalWrite(BUZZER_PIN, LOW);
    }

    // UPDATE `istatus` WITH DIRECT CAMERA WEB LINK!
    String tempStr = isnan(currentTemp) ? "--" : String(currentTemp, 1);
    String humStr  = isnan(currentHum)  ? "--" : String((int)currentHum);

    String alertSummary = "";
    if (iflame) alertSummary += "FIRE_CRITICAL! ";
    if (gasStatus == 'H') alertSummary += "GAS_CRITICAL! ";
    if (tempStatus == 'H') alertSummary += "HIGH_TEMP! ";
    if (iperson) alertSummary += "PERSON_IN_VIEW! ";
    if (alertSummary == "") alertSummary = "NORMAL";

    // Puts Camera Link at the very front so you can copy-paste from IoT Dashboard!
    istatus = "CAM: " + camWebLink + " | " +
              "T:" + tempStr + "C(" + tempStatus + ") H:" + humStr + "%(" + humStatus + ") | " +
              "Gas:" + String(currentGas) + "(" + gasStatus + ") | " +
              alertSummary;

    Serial.printf("[TELEMETRY] Cam: %s | T:%.1fC | H:%.1f%% | G:%d | Person:%s\n",
                  camWebLink.c_str(), currentTemp, currentHum, currentGas,
                  iperson ? "YES" : "NO");
  }

  // 4. Update 16x2 LCD Display (Every 400ms)
  if (currentMs - lastLcdUpdateMs >= LCD_UPDATE_MS) {
    lastLcdUpdateMs = currentMs;
    updateLcdScreen();
  }
}

// ==========================================
// 8. ARDUINO IOT CLOUD CALLBACK (icontrol)
// ==========================================
void onIcontrolChange() {
  String cmd = icontrol;
  cmd.trim();
  cmd.toUpperCase();

  Serial.printf("[IOT COMMAND RECEIVED] icontrol = '%s'\n", cmd.c_str());

  if (cmd == "AUTO" || cmd == "A") {
    startSquarePatrol();
  }
  else if (cmd == "FORWARD" || cmd == "F") {
    autoSquareActive = false;
    forward();
    Serial.println("[MANUAL] Robot moving Forward");
  }
  else if (cmd == "BACKWARD" || cmd == "BACK" || cmd == "B" || cmd == "REVERSE") {
    autoSquareActive = false;
    backward();
    Serial.println("[MANUAL] Robot moving Backward");
  }
  else if (cmd == "LEFT" || cmd == "L") {
    autoSquareActive = false;
    turnLeft();
    Serial.println("[MANUAL] Robot turning Left");
  }
  else if (cmd == "RIGHT" || cmd == "R") {
    autoSquareActive = false;
    turnRight();
    Serial.println("[MANUAL] Robot turning Right");
  }
  else if (cmd == "STOP" || cmd == "S") {
    autoSquareActive = false;
    stopRobot();
    Serial.println("[MANUAL] Robot Stopped");
  }
}
