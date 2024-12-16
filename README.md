# ZSX11H Motor Driver Controller

## Overview
This project involves building a motor driver controller for the ZSX11H motor driver using an Arduino. The system is capable of controlling **four motor drivers**, making it suitable for projects requiring precise multi-motor control, such as robotics, model cars, or industrial automation.

## Features
- Controls up to **four ZSX11H motor drivers**.
- Compatible with Arduino platforms.
- Supports **bidirectional motor control**.
- Configurable **speed control** using PWM signals.
- Easy integration with other microcontrollers or sensors.

## Components
- **Arduino Board** (e.g., Arduino Uno, Nano, or Mega).
- **ZSX11H Motor Drivers** (4 units).
- **Motors** (Brushless DC or brushed motors, compatible with ZSX11H).
- Power supply (suitable for the motor and driver requirements).
- Connecting wires and a breadboard (optional).

## Pin Configuration
Each ZSX11H motor driver has the following key input pins:
1. **PWM**: Controls the speed of the motor.
2. **DIR**: Sets the motor rotation direction (clockwise/counterclockwise).
3. **GND**: Common ground.
4. **Enable**: Activates the motor driver.

| Arduino Pin | Motor Driver Pin | Description                   |
|-------------|------------------|-------------------------------|
| D3          | PWM1             | Speed control for Motor 1    |
| D4          | DIR1             | Direction control for Motor 1|
| D5          | PWM2             | Speed control for Motor 2    |
| D6          | DIR2             | Direction control for Motor 2|
| D9          | PWM3             | Speed control for Motor 3    |
| D10         | DIR3             | Direction control for Motor 3|
| D11         | PWM4             | Speed control for Motor 4    |
| D12         | DIR4             | Direction control for Motor 4|

## Wiring Diagram
1. Connect each ZSX11H motor driver to its corresponding motor.
2. Wire the **PWM** and **DIR** pins of the motor drivers to the Arduino pins as per the pin configuration.
3. Connect all **GND** pins to a common ground.
4. Ensure the motor drivers are powered appropriately with an external power source.

## Software
The controller is programmed using the **Arduino IDE**. Below is a basic example of controlling four motor drivers:

## Usage
1. Upload the code to the Arduino using the **Arduino IDE**.
2. Power the Arduino and motor drivers.
3. Control the motors using the logic defined in the code.

## Applications
- **Robotics**: Control multiple robot arms or wheels.
- **Automated Vehicles**: Power model cars or drones.
- **Industrial Automation**: Drive conveyor belts or machinery.

## Future Improvements
- Add Bluetooth or WiFi control for wireless operation.
- Implement speed feedback using encoders.
- Design a custom PCB for compactness.

## License
This project is open-source. Feel free to modify and share it.

---

For questions or contributions, contact [Your Email Address] or visit [Your GitHub Profile].

