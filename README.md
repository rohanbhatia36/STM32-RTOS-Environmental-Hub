# Real-Time Environmental Monitoring Hub on STM32

This project is a multi-threaded environmental monitoring system built on an STM32F446RE microcontroller. It leverages the FreeRTOS real-time operating system to concurrently manage multiple hardware peripherals, read sensor data, and display it in real-time on an OLED screen.

---

## Key Features

- **Multi-Threading:** Utilizes FreeRTOS to run four independent tasks concurrently: display management, temperature sensing, light level sensing, and PWM control.
- **Real-Time Data Display:** Sensor data is continuously updated and displayed on a 1.3" SH1106 OLED display.
- **Concurrent Peripheral Management:** Demonstrates the use of I2C, SPI, ADC, and PWM peripherals all operating simultaneously without blocking each other.
- **Inter-Task Communication:** Employs FreeRTOS message queues for safe and efficient communication of sensor data between tasks.
- **Responsive Control:** A PWM-controlled LED's brightness changes in real-time based on the ambient light level.

---

## Hardware Components

| Component | Description |
| :--- | :--- |
| **Microcontroller** | STM32F446RE (on a Nucleo-64 board) |
| **Display** | HiLetgo 1.3" SPI 128x64 SH1106 OLED LCD Display |
| **Temperature Sensor** | HiLetgo BMP280 Atmospheric Pressure & Temperature Sensor (I2C) |
| **Light Sensor** | Generic Photoresistor (connected to an ADC pin) |
| **Indicator** | A standard LED (controlled by PWM) |

---

## Software and Technologies

- **Primary Language:** C
- **IDE:** STM32CubeIDE
- **Core Technologies:**
  - **FreeRTOS:** For real-time task scheduling and management.
  - **STM32 HAL:** The Hardware Abstraction Layer provided by STMicroelectronics for peripheral configuration.
  - **SPI & I2C:** Communication protocols for the display and temperature sensor.
  - **ADC & PWM:** For reading the analog light sensor and controlling the LED.

---

## System Architecture

The system is designed around four independent tasks that communicate using message queues, ensuring that no single task blocks the operation of another.

1.  **`StartReadSensorTask` (Temperature):**
    - Runs every 500ms.
    - Communicates with the BMP280 sensor over I2C.
    - Reads the current temperature, converts it to Fahrenheit, and places the value into the `temperatureQueue`.

2.  **`StartReadADCTask` (Light):**
    - Runs every 500ms.
    - Reads the analog voltage from the photoresistor using the ADC.
    - Places the raw digital value into two separate queues: `lightLevelQueue` (for the display) and `pwmQueue` (for the LED).

3.  **`StartControlPWMTask` (LED Control):**
    - Waits indefinitely for a new message to arrive in the `pwmQueue`.
    - Upon receiving a new light level, it calculates a corresponding duty cycle and updates the PWM signal to change the LED's brightness.

4.  **`StartDefaultTask` (Display Manager):**
    - Runs every 250ms.
    - Checks both the `temperatureQueue` and `lightLevelQueue` for new data.
    - It remembers the last valid reading to provide a stable, non-flickering display.
    - Formats the sensor data into strings, scales the light level to a 0-100% value, and updates the OLED display over SPI.
