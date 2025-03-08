#include <SoftwareSerial.h>

SoftwareSerial bluetooth(13, 4);  // RX, TX

// Motor 1 (Left)
const int PWM_L = 5;
const int DIR_L = 6;
const int BRAKE_L = 7;
const int STOP_L = 8;
int HALL_SENSOR_L = 3;

// Motor 2 (Right)
const int PWM_R = 9;
const int DIR_R = 10;
const int BRAKE_R = 11;
const int STOP_R = 12;
int HALL_SENSOR_R = 2;

volatile int pulseCountL = 0;
volatile int pulseCountR = 0;
int targetPulseL = 0;
int targetPulseR = 0;
boolean systemHalted = false;  // Flag to track if the system is halted
const float PULSES_PER_ROTATION_L = 44.0;
const float PULSES_PER_ROTATION_R = 45.0;

void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);

    pinMode(HALL_SENSOR_L, INPUT);
    pinMode(PWM_L, OUTPUT);
    pinMode(DIR_L, OUTPUT);
    pinMode(BRAKE_L, OUTPUT);
    pinMode(STOP_L, OUTPUT);
    digitalWrite(BRAKE_L, LOW);
    digitalWrite(STOP_L, LOW);

    pinMode(HALL_SENSOR_R, INPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(DIR_R, OUTPUT);
    pinMode(BRAKE_R, OUTPUT);
    pinMode(STOP_R, OUTPUT);
    digitalWrite(BRAKE_R, LOW);
    digitalWrite(STOP_R, LOW);

    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_L), recordPulseTimeL, RISING);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_R), recordPulseTimeR, RISING);
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        int motorID, targetPulse;

        if (input == "X") {
      // Halt all operations
      stopMotor();
      systemHalted = true;
      Serial.println("System halted.");
    }

    else if (input == "x") {
      // Resume all operations
      releaseMotor();
      systemHalted = false;
      Serial.println("System resumed.");
    }

        if (sscanf(input.c_str(), "%d:%d", &motorID, &targetPulse) == 2 && !systemHalted) {
            pulseCountL = 0;
            pulseCountR = 0;

            switch (motorID) {
                case 1:  // Move only Left Motor
                    targetPulseL = targetPulse;
                    targetPulseR = 0;  // Right motor should not move
                    digitalWrite(DIR_L, HIGH);
                    break;

                case 2:  // Move only Right Motor
                    targetPulseR = targetPulse;
                    targetPulseL = 0;  // Left motor should not move
                    digitalWrite(DIR_R, LOW);
                    break;

                case 3:  // Move both forward while keeping sync
                    targetPulseL = (targetPulse * PULSES_PER_ROTATION_L) / PULSES_PER_ROTATION_R;
                    targetPulseR = (targetPulse * PULSES_PER_ROTATION_R) / PULSES_PER_ROTATION_L;
                    digitalWrite(DIR_L, HIGH);
                    digitalWrite(DIR_R, LOW);
                    break;

                case 4:  // Move both backward while keeping sync
                    targetPulseL = (targetPulse * PULSES_PER_ROTATION_L) / PULSES_PER_ROTATION_R;
                    targetPulseR = (targetPulse * PULSES_PER_ROTATION_R) / PULSES_PER_ROTATION_L;
                    digitalWrite(DIR_L, LOW);
                    digitalWrite(DIR_R, HIGH);
                    break;
                case 5:
                    targetPulseL = (targetPulse * PULSES_PER_ROTATION_L) / PULSES_PER_ROTATION_R;
                    targetPulseR = (targetPulse * PULSES_PER_ROTATION_R) / PULSES_PER_ROTATION_L;
                    digitalWrite(DIR_L, HIGH);
                    digitalWrite(DIR_R, HIGH);
                    break;
                case 6:
                    targetPulseL = (targetPulse * PULSES_PER_ROTATION_L) / PULSES_PER_ROTATION_R;
                    targetPulseR = (targetPulse * PULSES_PER_ROTATION_R) / PULSES_PER_ROTATION_L;
                    digitalWrite(DIR_L, LOW);
                    digitalWrite(DIR_R, LOW);
                case 7:
                    targetPulseR = targetPulse;
                    targetPulseL = 0;  // Left motor should not move
                    digitalWrite(DIR_R, HIGH);
                    break;
                case 8:
                    targetPulseL = targetPulse;
                    targetPulseR = 0;  // Right motor should not move
                    digitalWrite(DIR_L, LOW);
            }
        }
    }

    // Control Left Motor
    if (pulseCountL < targetPulseL && targetPulseL > 0) {
        analogWrite(PWM_L, 12);
    } else {
        analogWrite(PWM_L, 0);
    }

    // Control Right Motor
    if (pulseCountR < targetPulseR && targetPulseR > 0) {
        analogWrite(PWM_R, 12);
    } else {
        analogWrite(PWM_R, 0);
    }
}

void recordPulseTimeL() {
    pulseCountL++;
}

void recordPulseTimeR() {
    pulseCountR++;
}

void stopMotor() {
  digitalWrite(BRAKE_R, HIGH);
  digitalWrite(BRAKE_L, HIGH);
  Serial.println("Brake applied. Motor stopped.");
  // motorStopped = true;
}

// Function to release the motor (disable brakes)
void releaseMotor() {
  digitalWrite(BRAKE_R, LOW);
  digitalWrite(BRAKE_L, LOW);

  Serial.println("Brake released. Motor free.");
  // motorStopped = false;
}

