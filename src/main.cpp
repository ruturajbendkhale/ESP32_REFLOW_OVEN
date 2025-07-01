#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1.h>
#include "PARAMETERS.h"
#include "PROFILE.h"

// Function declarations
void updateDisplay();
void handleSerialCommands();
void updateReflowProfile();
double calculateTargetTemperature(ReflowStatus& status);

// Initialize display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Note: DI (Data In) pin is not used by MAX6675 as it's read-only

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// PID Controller configuration
double Setpoint, Input, Output;
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;  // PID tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// SCR Control pin (PWM output)
#define SCR_CONTROL_PIN 16  // D0 on NodeMCU, you can change this to any PWM capable pin

// Temperature variables
double currentTemp = 0.0;
double targetTemp = DEFAULT_TARGET_TEMP;
unsigned long lastTempRead = 0;
unsigned long lastProfileUpdate = 0;
const unsigned long TEMP_READ_INTERVAL = TEMP_READ_INTERVAL;  // Read temperature every TEMP_READ_INTERVAL ms

// Display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = DISPLAY_UPDATE_INTERVAL;  // Update display every DISPLAY_UPDATE_INTERVAL ms

// Safety limits
const double MAX_TEMP = MAX_TEMP;  // Maximum safe temperature
const double MIN_TEMP = MIN_TEMP;    // Minimum temperature

// System state
bool heatingEnabled = true;
bool alarmState = false;
bool reflowInProgress = false;
ReflowStatus reflowStatus = {STAGE_IDLE, 0, AMBIENT_TEMP, 0, false};

// Function to calculate target temperature based on current profile stage
double calculateTargetTemperature(ReflowStatus& status) {
    double elapsedTimeInStage = (millis() - status.stageStartTime) / 1000.0; // Convert to seconds
    
    switch(status.currentStage) {
        case STAGE_PREHEAT:
            // Linear ramp from ambient to soak temperature
            return AMBIENT_TEMP + (PREHEAT_RAMP_RATE * elapsedTimeInStage);
            
        case STAGE_SOAK:
            // Maintain constant soak temperature
            return SOAK_TEMP;
            
        case STAGE_REFLOW:
            // Linear ramp from soak to peak temperature
            return SOAK_TEMP + (REFLOW_RAMP_RATE * elapsedTimeInStage);
            
        case STAGE_COOLING:
            // Linear cooling from peak temperature
            return PEAK_TEMP - (COOLING_RATE * elapsedTimeInStage);
            
        default:
            return AMBIENT_TEMP;
    }
}

// Function to update reflow profile state
void updateReflowProfile() {
    if (!reflowInProgress || alarmState) return;
    
    unsigned long currentTime = millis();
    double elapsedTimeInStage = (currentTime - reflowStatus.stageStartTime) / 1000.0; // seconds
    
    switch(reflowStatus.currentStage) {
        case STAGE_IDLE:
            if (heatingEnabled) {
                reflowStatus.currentStage = STAGE_PREHEAT;
                reflowStatus.stageStartTime = currentTime;
                Serial.println("Starting Preheat Stage");
            }
            break;
            
        case STAGE_PREHEAT:
            if (currentTemp >= SOAK_TARGET_TEMP) {
                reflowStatus.currentStage = STAGE_SOAK;
                reflowStatus.stageStartTime = currentTime;
                Serial.println("Entering Soak Stage");
            }
            break;
            
        case STAGE_SOAK:
            if (elapsedTimeInStage >= SOAK_DURATION) {
                reflowStatus.currentStage = STAGE_REFLOW;
                reflowStatus.stageStartTime = currentTime;
                Serial.println("Starting Reflow Stage");
            }
            break;
            
        case STAGE_REFLOW:
            if (currentTemp >= PEAK_TEMP || elapsedTimeInStage >= TIME_ABOVE_LIQUIDUS) {
                reflowStatus.currentStage = STAGE_COOLING;
                reflowStatus.stageStartTime = currentTime;
                Serial.println("Starting Cooling Stage");
            }
            break;
            
        case STAGE_COOLING:
            if (currentTemp <= SAFE_HANDLING_TEMP) {
                reflowStatus.currentStage = STAGE_COMPLETE;
                reflowStatus.isComplete = true;
                reflowInProgress = false;
                heatingEnabled = false;
                Serial.println("Reflow Process Complete");
            }
            break;
            
        case STAGE_COMPLETE:
            // Do nothing, wait for manual reset
            break;
    }
    
    // Update target temperature based on current stage
    reflowStatus.targetTemp = calculateTargetTemperature(reflowStatus);
    Setpoint = reflowStatus.targetTemp;
    
    // Update elapsed time
    reflowStatus.elapsedTime = (currentTime - reflowStatus.stageStartTime) / 1000.0;
}

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
    for(;;); // Keep this one as display is essential
  }
  
  // Test display
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  
  // Test pattern: Text
  display.setCursor(0,0);
  display.println("Display");
  display.println("Test");
  display.display();
  delay(DISPLAY_MESSAGE_DELAY);
  
  // Regular initialization message
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Reflow Oven v1.0");
  display.println("Initializing...");
  display.display();
  
  // Initialize MAX6675
  Serial.println("Initializing MAX6675...");
  delay(SENSOR_INIT_DELAY);
  
  // Test temperature reading
  currentTemp = thermocouple.readCelsius();
  if (isnan(currentTemp)) {
    Serial.println("Warning: Could not read temperature from MAX6675!");
    Serial.println("Check wiring and ensure sensor is connected.");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("WARNING:");
    display.println("MAX6675 not found");
    display.println("Check wiring");
    display.println("");
    display.println("System will retry");
    display.println("automatically");
    display.display();
    delay(ERROR_MESSAGE_DELAY);
    alarmState = true;
  } else {
    Serial.print("Initial temperature: ");
    Serial.print(currentTemp);
    Serial.println("°C");
    alarmState = false;
  }
  
  // Initialize PID
  Setpoint = targetTemp;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // PWM output range (0-255)
  myPID.SetSampleTime(TEMP_READ_INTERVAL);
  
  // Initialize SCR control pin
  pinMode(SCR_CONTROL_PIN, OUTPUT);
  analogWrite(SCR_CONTROL_PIN, 0);  // Start with heater off
  
  Serial.println("System initialized successfully!");
  Serial.println("Commands:");
  Serial.println("  s<temp> - Set target temperature (e.g., s150)");
  Serial.println("  h - Toggle heating on/off");
  Serial.println("  p<value> - Set Kp value");
  Serial.println("  i<value> - Set Ki value");
  Serial.println("  d<value> - Set Kd value");
  Serial.println("  start - Start reflow process");
  Serial.println("  stop - Stop reflow process");
  Serial.println("  status - Show system status");
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
  display.print(targetTemp, 1);
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
    display.print("Heating: ");
    display.print(Output > 0 ? "ON" : "OFF");
  } else {
    display.print("DISABLED");
  }
  
  // Temperature trend indicator (simple)
  static double lastDisplayTemp = currentTemp;
  display.setCursor(100, 15);
  display.setTextSize(1);
  if (currentTemp > lastDisplayTemp + 0.5) {
    display.print("^");
  } else if (currentTemp < lastDisplayTemp - 0.5) {
    display.print("v");
  } else {
    display.print("-");
  }
  lastDisplayTemp = currentTemp;
  
  display.display();
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "start") {
            // Start reflow process
            reflowInProgress = true;
            heatingEnabled = true;
            reflowStatus = {STAGE_IDLE, millis(), AMBIENT_TEMP, 0, false};
            Serial.println("Starting reflow process");
        } else if (command == "stop") {
            // Stop reflow process
            reflowInProgress = false;
            heatingEnabled = false;
            reflowStatus.currentStage = STAGE_IDLE;
            Serial.println("Reflow process stopped");
        } else if (command == "status") {
            // Print current status
            Serial.println("=== Reflow Status ===");
            Serial.print("Stage: ");
            switch(reflowStatus.currentStage) {
                case STAGE_IDLE: Serial.println("IDLE"); break;
                case STAGE_PREHEAT: Serial.println("PREHEAT"); break;
                case STAGE_SOAK: Serial.println("SOAK"); break;
                case STAGE_REFLOW: Serial.println("REFLOW"); break;
                case STAGE_COOLING: Serial.println("COOLING"); break;
                case STAGE_COMPLETE: Serial.println("COMPLETE"); break;
            }
            Serial.print("Current Temp: ");
            Serial.print(currentTemp);
            Serial.println("°C");
            Serial.print("Target Temp: ");
            Serial.print(reflowStatus.targetTemp);
            Serial.println("°C");
            Serial.print("Elapsed Time: ");
            Serial.print(reflowStatus.elapsedTime);
            Serial.println("s");
        } else if (command.startsWith("s")) {
            // Existing temperature set command
            double newTarget = command.substring(1).toDouble();
            if (newTarget >= MIN_TEMP && newTarget <= MAX_TEMP) {
                targetTemp = newTarget;
                Setpoint = targetTemp;
                Serial.print("Target temperature set to: ");
                Serial.print(targetTemp);
                Serial.println("°C");
            } else {
                Serial.println("Error: Temperature out of range!");
            }
        } else if (command == "h") {
            // Existing heating toggle
            heatingEnabled = !heatingEnabled;
            if (!heatingEnabled) {
                reflowInProgress = false;
                reflowStatus.currentStage = STAGE_IDLE;
            }
            Serial.print("Heating ");
            Serial.println(heatingEnabled ? "enabled" : "disabled");
        } else {
            // Help menu
            Serial.println("Available commands:");
            Serial.println("  start - Start reflow process");
            Serial.println("  stop - Stop reflow process");
            Serial.println("  status - Show current status");
            Serial.println("  s<temp> - Set manual target temperature");
            Serial.println("  h - Toggle heating on/off");
        }
    }
}

void loop() {
    // Read temperature with error recovery
    if (millis() - lastTempRead >= TEMP_READ_INTERVAL) {
        double tempReading = thermocouple.readCelsius();
        lastTempRead = millis();
        
        // Check for sensor errors with recovery
        if (isnan(tempReading)) {
            if (!alarmState) {  // Only print message on transition to error state
                Serial.println("Error: Temperature sensor disconnected!");
                Serial.println("Waiting for sensor reconnection...");
            }
            alarmState = true;
            // Don't update currentTemp when reading fails
        } else {
            if (alarmState) {  // Sensor recovered
                Serial.println("Sensor connection restored!");
            }
            alarmState = false;
            currentTemp = tempReading;
            Input = currentTemp;
        }
        
        // Safety check
        if (currentTemp > MAX_TEMP) {
            heatingEnabled = false;
            alarmState = true;
            Serial.println("ALARM: Temperature too high! Heater disabled.");
        }
    }
    
    // Update reflow profile if active
    if (millis() - lastProfileUpdate >= PROFILE_CHECK_INTERVAL) {
        updateReflowProfile();
        lastProfileUpdate = millis();
    }
    
    // PID calculation and SCR control - only if sensor is working and not in alarm
    if (heatingEnabled && !alarmState) {
        myPID.Compute();
        analogWrite(SCR_CONTROL_PIN, (int)Output);
    } else {
        Output = 0;
        analogWrite(SCR_CONTROL_PIN, 0);  // Turn off heater
    }
    
    // Update display
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    // Handle serial commands
    handleSerialCommands();
    
    // Print status to serial (optional, can be removed for production)
    static unsigned long lastSerialPrint = 0;
    if (millis() - lastSerialPrint >= 2000) {  // Print every 2 seconds
        Serial.print("Temp: ");
        Serial.print(currentTemp, 1);
        Serial.print("°C, Target: ");
        Serial.print(reflowStatus.targetTemp, 1);
        Serial.print("°C, Output: ");
        Serial.print(Output, 0);
        Serial.print("/255, Stage: ");
        switch(reflowStatus.currentStage) {
            case STAGE_IDLE: Serial.print("IDLE"); break;
            case STAGE_PREHEAT: Serial.print("PREHEAT"); break;
            case STAGE_SOAK: Serial.print("SOAK"); break;
            case STAGE_REFLOW: Serial.print("REFLOW"); break;
            case STAGE_COOLING: Serial.print("COOLING"); break;
            case STAGE_COMPLETE: Serial.print("COMPLETE"); break;
        }
        Serial.println();
        lastSerialPrint = millis();
    }
}