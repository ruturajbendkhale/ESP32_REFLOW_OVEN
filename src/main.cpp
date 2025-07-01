#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1.h>
#include "PARAMETERS.h"
#include "PROFILE.h"
#include "ReflowController.h"

// Function declarations
void updateDisplay();
void handleSerialCommands();

// Initialize display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Initialize MAX6675
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Initialize Reflow Controller
ReflowController reflowController;

// PID Controller configuration
double Setpoint, Input, Output;
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Temperature variables
double currentTemp = 0.0;
unsigned long lastTempRead = 0;

// Display update timing
unsigned long lastDisplayUpdate = 0;

// System state
bool heatingEnabled = true;
bool alarmState = false;



void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(STARTUP_DELAY);
    Serial.println("\nESP32 Reflow Oven PID Controller Starting...");
    
    // Initialize I2C for display
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize display
    Serial.println("Initializing Display...");
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed! Check wiring:"));
        Serial.println(F("- SDA connected to GPIO 12 (D6)"));
        Serial.println(F("- SCL connected to GPIO 14 (D5)"));
        Serial.println(F("- Display VCC to 3.3V"));
        Serial.println(F("- Display GND to GND"));
        for(;;);
    }
    
    // Initialize components
    reflowController.begin();
    
    // Initialize PID
    Setpoint = AMBIENT_TEMP;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTime(TEMP_READ_INTERVAL);
    
    // Initialize SCR control pin
    pinMode(SCR_CONTROL_PIN, OUTPUT);
    analogWrite(SCR_CONTROL_PIN, 0);
    
    Serial.println("System initialized successfully!");
    Serial.println("Commands:");
    Serial.println("  start - Start reflow process");
    Serial.println("  stop - Stop reflow process");
    Serial.println("  status - Show current status");
    Serial.println("  h - Toggle heating on/off");
}

void updateDisplay() {
    display.clearDisplay();
    
    // Title
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Reflow Oven PID");
    
    // Draw a line
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    
    // Current temperature (large font)
    display.setTextSize(2);
    display.setCursor(0, 15);
    if (!alarmState) {
        display.print(currentTemp, 1);
        display.print("C");
    } else {
        display.print("ERROR");
    }
    
    // Target temperature
    display.setTextSize(1);
    display.setCursor(0, 35);
    display.print("Target: ");
    display.print(reflowController.getTargetTemperature(), 1);
    display.print("C");
    
    // PID Output
    display.setCursor(0, 45);
    display.print("Power: ");
    display.print((Output/255.0)*100, 0);
    display.print("%");
    
    // Status indicators
    display.setCursor(0, 55);
    if (alarmState) {
        display.print("ALARM!");
    } else if (heatingEnabled) {
        display.print("Stage: ");
        display.print(ReflowController::getStageString(reflowController.getCurrentStage()));
    } else {
        display.print("DISABLED");
    }
    
    display.display();
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "start") {
            heatingEnabled = true;
            reflowController.start();
        } else if (command == "stop") {
            heatingEnabled = false;
            reflowController.stop();
        } else if (command == "status") {
            Serial.println("=== System Status ===");
            Serial.print("Current Temp: ");
            Serial.print(currentTemp);
            Serial.println("°C");
            Serial.print("Target Temp: ");
            Serial.print(reflowController.getTargetTemperature());
            Serial.println("°C");
            Serial.print("Stage: ");
            Serial.println(ReflowController::getStageString(reflowController.getCurrentStage()));
            Serial.print("Elapsed Time: ");
            Serial.print(reflowController.getElapsedTime());
            Serial.println("s");
            Serial.print("Heating: ");
            Serial.println(heatingEnabled ? "ON" : "OFF");
        } else if (command == "h") {
            heatingEnabled = !heatingEnabled;
            if (!heatingEnabled) {
                reflowController.stop();
            }
            Serial.print("Heating ");
            Serial.println(heatingEnabled ? "enabled" : "disabled");
        } else {
            Serial.println("Available commands:");
            Serial.println("  start - Start reflow process");
            Serial.println("  stop - Stop reflow process");
            Serial.println("  status - Show current status");
            Serial.println("  h - Toggle heating on/off");
        }
    }
}

void loop() {
    // Read temperature with error recovery
    if (millis() - lastTempRead >= TEMP_READ_INTERVAL) {
        double tempReading = thermocouple.readCelsius();
        lastTempRead = millis();
        
        if (isnan(tempReading)) {
            if (!alarmState) {
                Serial.println("Error: Temperature sensor disconnected!");
                Serial.println("Waiting for sensor reconnection...");
            }
            alarmState = true;
        } else {
            if (alarmState) {
                Serial.println("Sensor connection restored!");
            }
            alarmState = false;
            currentTemp = tempReading;
            Input = currentTemp;
            
            // Update reflow controller
            reflowController.update(currentTemp);
        }
        
        // Safety check
        if (currentTemp > MAX_TEMP) {
            heatingEnabled = false;
            alarmState = true;
            reflowController.stop();
            Serial.println("ALARM: Temperature too high! Heater disabled.");
        }
    }
    
    // Update target temperature from reflow controller if running
    if (reflowController.isRunning()) {
        Setpoint = reflowController.getTargetTemperature();
    }
    
    // PID calculation and SCR control
    if (heatingEnabled && !alarmState) {
        myPID.Compute();
        analogWrite(SCR_CONTROL_PIN, (int)Output);
    } else {
        Output = 0;
        analogWrite(SCR_CONTROL_PIN, 0);
    }
    
    // Update display
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    // Handle serial commands
    handleSerialCommands();
}