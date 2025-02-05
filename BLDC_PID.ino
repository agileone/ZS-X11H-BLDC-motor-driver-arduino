#include <SoftwareSerial.h>

SoftwareSerial bluetooth(13, 4);  // RX, TX for Bluetooth communication

// Motor 1 Pins
int DIR_PIN_M1 = 6;         // Motor 1 Direction pin
int PWM_PIN_M1 = 5;         // Motor 1 PWM control pin (490Hz)
int HALL_SENSOR_M1 = 3;     // Motor 1 Hall sensor interrupt feedback pin

// Motor 2 Pins
int DIR_PIN_M2 = 10;         // Motor 2 Direction pin
int PWM_PIN_M2 = 9;         // Motor 2 PWM control pin (490Hz)
int HALL_SENSOR_M2 = 2;     // Motor 2 Hall sensor interrupt feedback pin

// PID Variables
volatile float tic1 = 0, tac1 = 0, feedback1 = 0.0;
volatile float tic2 = 0, tac2 = 0, feedback2 = 0.0;

float now, prvTime, dt;
float P1 = 0.00, I1 = 0.00, D1 = 0.00, error1 = 0.00, prevErr1 = 0.00, errSum1 = 0.00, pid1 = 0.00; // Motor 1 PID
float P2 = 0.00, I2 = 0.00, D2 = 0.00, error2 = 0.00, prevErr2 = 0.00, errSum2 = 0.00, pid2 = 0.00; // Motor 2 PID

// PID Constants - Adjust for best performance
float kp = 0.15, ki = 0.7, kd = 0.001;
float targetSpeed = 20;   // Target speed (tune based on your system)
float maxPWM = 255, minPWM = 0; // PWM limits
float fb_min = 104, fb_max = 46, trgt_min = 12, trgt_max = 25; // Feedback mapping

void setup() {
    Serial.begin(115200);
    bluetooth.begin(9600);  // Start Bluetooth communication at 9600 baud rate

    // Motor 1 Pins
    pinMode(DIR_PIN_M1, OUTPUT);
    pinMode(PWM_PIN_M1, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_M1), measureSpeed1, RISING); // Interrupt for Motor 1 feedback

    // Motor 2 Pins
    pinMode(DIR_PIN_M2, OUTPUT);
    pinMode(PWM_PIN_M2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_M2), measureSpeed2, RISING); // Interrupt for Motor 2 feedback

    // Set initial motor directions (optional, you can change them during runtime)
    digitalWrite(DIR_PIN_M1, LOW);  // Motor 1 initial direction
    digitalWrite(DIR_PIN_M2, LOW);  // Motor 2 initial direction
}

void loop() {
    now = millis();
    dt = (now - prvTime) / 1000.0; // Time step for PID calculation

    if (dt < 0.001) dt = 0.001; // Prevent division by zero errors
    prvTime = now;

    // Calculate PID for both motors
    pid1 = PID(feedback1, targetSpeed, P1, I1, D1, error1, prevErr1, errSum1);  
    pid2 = PID(feedback2, targetSpeed, P2, I2, D2, error2, prevErr2, errSum2);

    // Constrain the PID output to the PWM limits
    pid1 = constrain(pid1, minPWM, maxPWM);
    pid2 = constrain(pid2, minPWM, maxPWM);

    // Apply PWM to both motors
    analogWrite(PWM_PIN_M1, round(pid1));  // Apply motor 1 speed
    analogWrite(PWM_PIN_M2, round(pid2));  // Apply motor 2 speed

    // Optional: Send feedback data to Bluetooth for monitoring
    bluetooth.print("Motor 1 Speed: ");
    bluetooth.print(feedback1);
    bluetooth.print("\tMotor 1 PID Output: ");
    bluetooth.println(pid1);
    
    bluetooth.print("Motor 2 Speed: ");
    bluetooth.print(feedback2);
    bluetooth.print("\tMotor 2 PID Output: ");
    bluetooth.println(pid2);
}

void measureSpeed1() {
    noInterrupts();  // Disable interrupts to prevent timing issues
    tic1 = millis();
    feedback1 = tic1 - tac1;
    tac1 = tic1;
    feedback1 = map(feedback1, fb_min, fb_max, trgt_min, trgt_max);
    interrupts();  // Re-enable interrupts
}

void measureSpeed2() {
    noInterrupts();
    tic2 = millis();
    feedback2 = tic2 - tac2;
    tac2 = tic2;
    feedback2 = map(feedback2, fb_min, fb_max, trgt_min, trgt_max);
    interrupts();
}

// PID Function - Common for both motors
float PID(float feedback, float targetSpeed, float &P, float &I, float &D, float &error, float &prevErr, float &errSum) {
    noInterrupts();  
    error = targetSpeed - feedback;  // Calculate error for target speed
    interrupts();

    P = kp * error; // Proportional term
    I += ki * error * dt; // Integral term
    D = kd * (error - prevErr) / dt; // Derivative term
    prevErr = error; // Update previous error

    return P + I + D; // Return the PID output
}
