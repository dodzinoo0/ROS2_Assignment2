/*
 * Robotic Arm Servo Controller — Arduino Uno / Nano / Mega Firmware
 *
 * Receives newline-terminated ASCII CSV frames over USB Serial at 115200 baud:
 *   S,<servo1_deg>,<servo2_deg>\n
 *   (also accepts up to 4 angles: S,<s1>,<s2>,<s3>,<s4>\n)
 *
 * Each angle is clamped to [0, 180] degrees and written via PWM using Servo.h.
 *
 * Wiring:
 *   D9  -> Servo 1 signal (joint1: base rotation)
 *   D10 -> Servo 2 signal (joint2: shoulder pitch)
 *   D11 -> Servo 3 signal (joint3: elbow pitch, optional)
 *   D6  -> Servo 4 signal (joint4: wrist pitch, optional)
 *   External 5V supply -> Servo VCC & GND (common GND shared with Arduino GND)
 */

#include <Servo.h>

static const unsigned long BAUD_RATE = 115200;
static const int NUM_SERVOS = 4;
static const int SERVO_PINS[NUM_SERVOS] = {9, 10, 11, 6};
static const int DEFAULT_ANGLE = 90;

Servo servos[NUM_SERVOS];
int currentAngles[NUM_SERVOS] = {DEFAULT_ANGLE, DEFAULT_ANGLE, DEFAULT_ANGLE, DEFAULT_ANGLE};

String rxBuffer;

static int clampAngle(int deg) {
  if (deg < 0) return 0;
  if (deg > 180) return 180;
  return deg;
}

static void processLine(String line) {
  line.trim();
  if (line.length() < 3 || line.charAt(0) != 'S' || line.charAt(1) != ',') {
    return;
  }

  // Parse comma-separated integer angles after "S,"
  int startIdx = 2;
  int servoIdx = 0;

  while (servoIdx < NUM_SERVOS && startIdx < (int)line.length()) {
    int commaIdx = line.indexOf(',', startIdx);
    String token;
    if (commaIdx == -1) {
      token = line.substring(startIdx);
      startIdx = line.length();
    } else {
      token = line.substring(startIdx, commaIdx);
      startIdx = commaIdx + 1;
    }

    token.trim();
    if (token.length() > 0) {
      int angle = clampAngle(token.toInt());
      currentAngles[servoIdx] = angle;
      servos[servoIdx].write(angle);
      servoIdx++;
    }
  }

  if (servoIdx > 0) {
    Serial.print("OK");
    for (int i = 0; i < servoIdx; i++) {
      Serial.print(',');
      Serial.print(currentAngles[i]);
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  rxBuffer.reserve(64);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(DEFAULT_ANGLE);
  }

  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      processLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      if (rxBuffer.length() < 63) {
        rxBuffer += c;
      } else {
        // Prevent overflow on malformed input
        rxBuffer = "";
      }
    }
  }
}
