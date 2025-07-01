#ifndef PARAMETERS_H
#define PARAMETERS_H

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// I2C Pins for ESP32
#define I2C_SDA 12  // GPIO12 (D6)
#define I2C_SCL 14  // GPIO14 (D5)

// MAX6675 configuration - SPI pins
#define thermoDO 19    // SO (Serial Out)
#define thermoCS 5     // CS (Chip Select)
#define thermoCLK 18   // SCK (Serial Clock)

// SCR Control pin
#define SCR_CONTROL_PIN 16  // PWM output pin

// Temperature configuration
#define DEFAULT_TARGET_TEMP 25.0   // Default setpoint in Celsius
#define MAX_TEMP 300.0            // Maximum safe temperature
#define MIN_TEMP 0.0              // Minimum temperature
#define TEMP_READ_INTERVAL 250    // Read temperature every 250ms

// Display update timing
#define DISPLAY_UPDATE_INTERVAL 500  // Update display every 500ms

// PID parameters
#define DEFAULT_KP 2.0
#define DEFAULT_KI 5.0
#define DEFAULT_KD 1.0

// Serial communication
#define SERIAL_BAUD_RATE 115200

// System delays
#define STARTUP_DELAY 1000        // Delay after serial initialization (ms)
#define SENSOR_INIT_DELAY 500     // Delay for MAX6675 to stabilize (ms)
#define DISPLAY_MESSAGE_DELAY 2000 // Delay for displaying messages (ms)
#define ERROR_MESSAGE_DELAY 3000   // Delay for error messages (ms)

#endif // PARAMETERS_H 