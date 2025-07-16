#ifndef PARAMETERS_H
#define PARAMETERS_H

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// encoder configuration
#define ENCODER_CLK D4
#define ENCODER_DT D3
// #define ENCODER_SW D4

// I2C Pins for NodeMCU ESP8266
#define I2C_SDA D2  // GPIO4 (D2)
#define I2C_SCL D1  // GPIO5 (D1)

// MAX6675 configuration - SPI pins
#define thermoDO D7    // SO (Serial Out) - GPIO13 (D7)
#define thermoCS D8    // CS (Chip Select) - GPIO15 (D8)
#define thermoCLK D5   // SCK (Serial Clock) - GPIO14 (D5)
// SCR Control pin
#define SCR_CONTROL_PIN D6  // PWM output pin - GPIO12 (D6)

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

// Cycle Burst Control parameters
#define CYCLE_BURST_PERIOD 500    // Total period for one cycle burst (ms)
#define MIN_BURST_TIME 100         // Minimum ON/OFF time to prevent too rapid switching (ms)

#endif // PARAMETERS_H 