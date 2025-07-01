#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1.h>

// Function declarations
void updateDisplay();
void handleSerialCommands();

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// I2C Pins for ESP32
#define I2C_SDA 12  // GPIO12 (D6)
#define I2C_SCL 14  // GPIO14 (D5)

// Initialize display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX6675 configuration - SPI pins (changed to avoid I2C overlap)
#define thermoDO 19    // SO (Serial Out)
#define thermoCS 5     // CS (Chip Select)
#define thermoCLK 18   // SCK (Serial Clock)
// Note: DI (Data In) pin is not used by MAX6675 as it's read-only

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// PID Controller configuration
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;  // PID tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// SCR Control pin (PWM output)
#define SCR_CONTROL_PIN 16  // D0 on NodeMCU, you can change this to any PWM capable pin

// Temperature variables
double currentTemp = 0.0;
double targetTemp = 25.0;  // Default setpoint in Celsius
unsigned long lastTempRead = 0;
const unsigned long TEMP_READ_INTERVAL = 250;  // Read temperature every 250ms

// Display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;  // Update display every 500ms

// Safety limits
const double MAX_TEMP = 300.0;  // Maximum safe temperature
const double MIN_TEMP = 0.0;    // Minimum temperature

// System state
bool heatingEnabled = true;
bool alarmState = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
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
  delay(2000);
  
  // Regular initialization message
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Reflow Oven v1.0");
  display.println("Initializing...");
  display.display();
  
  // Initialize MAX6675
  Serial.println("Initializing MAX6675...");
  delay(500);  // Wait for MAX6675 to stabilize
  
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
    delay(3000);  // Show message for 3 seconds
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
  myPID.SetSampleTime(250);       // PID calculation every 250ms
  
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
    
    if (command.startsWith("s")) {
      // Set temperature command: s150
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
    }
    else if (command == "h") {
      // Toggle heating
      heatingEnabled = !heatingEnabled;
      Serial.print("Heating ");
      Serial.println(heatingEnabled ? "enabled" : "disabled");
      if (!heatingEnabled) {
        analogWrite(SCR_CONTROL_PIN, 0);
      }
    }
    else if (command.startsWith("p")) {
      // Set Kp value
      double newKp = command.substring(1).toDouble();
      if (newKp >= 0) {
        Kp = newKp;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print("Kp set to: ");
        Serial.println(Kp);
      }
    }
    else if (command.startsWith("i")) {
      // Set Ki value
      double newKi = command.substring(1).toDouble();
      if (newKi >= 0) {
        Ki = newKi;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print("Ki set to: ");
        Serial.println(Ki);
      }
    }
    else if (command.startsWith("d")) {
      // Set Kd value
      double newKd = command.substring(1).toDouble();
      if (newKd >= 0) {
        Kd = newKd;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.print("Kd set to: ");
        Serial.println(Kd);
      }
    }
    else if (command == "status") {
      // Print detailed status
      Serial.println("=== System Status ===");
      Serial.print("Current Temperature: ");
      Serial.print(currentTemp);
      Serial.println("°C");
      Serial.print("Target Temperature: ");
      Serial.print(targetTemp);
      Serial.println("°C");
      Serial.print("PID Output: ");
      Serial.print(Output);
      Serial.println("/255");
      Serial.print("PID Parameters - Kp: ");
      Serial.print(Kp);
      Serial.print(", Ki: ");
      Serial.print(Ki);
      Serial.print(", Kd: ");
      Serial.println(Kd);
      Serial.print("Heating Enabled: ");
      Serial.println(heatingEnabled ? "Yes" : "No");
      Serial.print("Alarm State: ");
      Serial.println(alarmState ? "ACTIVE" : "Normal");
    }
    else {
      Serial.println("Unknown command. Available commands:");
      Serial.println("  s<temp> - Set target temperature");
      Serial.println("  h - Toggle heating");
      Serial.println("  p<value> - Set Kp");
      Serial.println("  i<value> - Set Ki");
      Serial.println("  d<value> - Set Kd");
      Serial.println("  status - Show system status");
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
    Serial.print(targetTemp, 1);
    Serial.print("°C, Output: ");
    Serial.print(Output, 0);
    Serial.print("/255, Heating: ");
    Serial.println(heatingEnabled ? "ON" : "OFF");
    lastSerialPrint = millis();
  }
}