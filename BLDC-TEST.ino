#include <SoftwareSerial.h>

// Motor control pins
const int PWM_L = 5;  // PWM for left motors
const int DIR_L = 6;  // Direction for left motors
const int BRAKE_L = 7;  // Brake for left motors
const int STOP_L = 8;  // Stop for left motors

// Right side motor control pins
const int PWM_R = 9;  // PWM for right motors
const int DIR_R = 10;  // Direction for right motors
const int BRAKE_R = 11;  // Brake for right motors
const int STOP_R = 12;  // Stop for right motors

// const int SPEED_PULSE_IN = 2;
// unsigned long pulse_time = 0; // Time of last pulse
// unsigned long pulse_period = 0; // Period of pulse in microseconds
// unsigned int pulse_frequency = 0; // Pulse frequency in Hz
// volatile int pulse_rpm = 0; // RPM of motor



boolean motorStopped = false;  // Track if motor is stopped
boolean systemHalted = false;  // Flag to track if the system is halted

// Bluetooth
SoftwareSerial bluetooth(3, 4);  // RX, TX

// Variables
int speedVal = 0;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Set motor control pins
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(BRAKE_L, OUTPUT);
  pinMode(STOP_L, OUTPUT);

  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(BRAKE_R, OUTPUT);
  pinMode(STOP_R, OUTPUT);

//   pinMode(SPEED_PULSE_IN, INPUT_PULLUP);

// // Set up interrupt to capture speed pulse
//   attachInterrupt(digitalPinToInterrupt(SPEED_PULSE_IN), pulse_captured, RISING);

  // TCCR0A = (1 << WGM00) | (1 << WGM01); // Set to Fast PWM
  // TCCR0B = (1 << CS01); // Prescaler set to 8 (gives ~3.9 kHz with 16 MHz clock)

  digitalWrite(PWM_L, LOW);   // Set PWM to 0 (no speed)
  digitalWrite(PWM_R, LOW);   // Set PWM to 0 (no speed)

  digitalWrite(DIR_R, LOW);   // Set direction to neutral
  digitalWrite(DIR_L, LOW);   // Set direction to neutral

  digitalWrite(BRAKE_L, HIGH); // Apply brake
  digitalWrite(BRAKE_R, HIGH); // Apply brake  

  digitalWrite(STOP_L, HIGH);  // Stop the motor
  digitalWrite(STOP_R, HIGH);  // Stop the motor

  // Start with motor stopped
  stopMotor();
}

void loop() {
  // Read Bluetooth command if available
  if (bluetooth.available()) {
    char command = bluetooth.read();

    // Handle halt and resume commands
    if (command == 'X') {
      // Halt all operations
      stopMotor();
      systemHalted = true;
      Serial.println("System halted.");
    } 
    
    else if (command == 'x') {
      // Resume all operations
      releaseMotor();
      systemHalted = false;
      Serial.println("System resumed.");
    }

    // If system is not halted, process other commands
    if (!systemHalted) {
      switch (command) {
        case 'F':  // Move Forward
          moveForward();
          break;
        case 'B':  // Move Backward
          moveBackward();
          break;
        case 'L':  // Turn Left
          turnLeft();
          break;
        case 'R':  // Turn Right
          turnRight();
          break;
        case 'S':
          applyBrake();
          break;
        case '0':
          speedVal = 0;
          Serial.println("speed set 0");
          break;
        case '1':
          speedVal = 10;
          Serial.println("speed set 10");
        break;
        case '2':
          speedVal = 20;
          Serial.println("speed set 20");
        break;
        case '3':
          speedVal = 30;
          Serial.println("speed set 30");
        break;
        case '4':
          speedVal = 40;
          Serial.println("speed set 40");
        break;
        case '5':
          speedVal = 50;
          Serial.println("speed set 50");
        break;
        case '6':
          speedVal = 60;
          Serial.println("speed set 60");
        break;
        case '7':
          speedVal = 70;
          Serial.println("speed set 70");
        break;
        case '8':
          speedVal = 80;
          Serial.println("speed set 80");
        break;
        case '9':
          speedVal = 90;
          Serial.println("speed set 90");
        break;
        case 'q':  // Set speed to maximum
          speedVal = 180;
          Serial.println("Speed set to 100%");
          break;
        default:
          Serial.println("Invalid Command");
          break;
      }

      // Apply speed control to motor
      
    }
      analogWrite(PWM_R, speedVal);
      analogWrite(PWM_L, speedVal);
  }
  // bluetooth.println(pulse_rpm);
}

// void pulse_captured() {
// // Calculate pulse period in microseconds
// pulse_period = micros() - pulse_time;
// pulse_time = micros();

// // Calculate pulse frequency in Hz
// pulse_frequency = 1000000 / pulse_period;

// // Calculate motor RPM
// pulse_rpm = pulse_frequency / 2;
// }



// Function to stop the motor (apply brakes)
void stopMotor() {
  digitalWrite(BRAKE_R, HIGH);
  digitalWrite(BRAKE_L, HIGH);  
  Serial.println("Brake applied. Motor stopped.");
  motorStopped = true;
}

// Function to release the motor (disable brakes)
void releaseMotor() {
  digitalWrite(BRAKE_R, LOW);
  digitalWrite(BRAKE_L, LOW);  

  Serial.println("Brake released. Motor free.");
  motorStopped = false;
}

// Function to move forward
void moveForward() {

  digitalWrite(DIR_L, LOW);  // Set direction to forward
  digitalWrite(DIR_R, HIGH);  // Set direction to forward


  digitalWrite(STOP_R, LOW);
  digitalWrite(STOP_L, LOW);
  Serial.println("Moving forward");
}

// Function to move backward
void moveBackward() {
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, LOW);

  digitalWrite(STOP_R, LOW);
  digitalWrite(STOP_L, LOW);
  Serial.println("Moving backward");
}

void turnLeft(){
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW); 

  digitalWrite(STOP_R, LOW);
  digitalWrite(STOP_L, LOW);
  Serial.println("Turning left");  
}

void turnRight() {
  // Right motors stop, left motors move forward
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, HIGH); 


  digitalWrite(STOP_R, LOW);
  digitalWrite(STOP_L, LOW);

  Serial.println("Turning Right");  
}

// Function to apply brakes
void applyBrake() {
  digitalWrite(STOP_R, HIGH);
  digitalWrite(STOP_L, HIGH);
  Serial.println("Brakes applied.");
}
