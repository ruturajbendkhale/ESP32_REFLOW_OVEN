#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <RotaryEncoder.h>


#include "PARAMETERS.h"
#include "PROFILE.h"
#include "ReflowController.h"

// WiFi credentials
const char* ssid = "ross-srv";     // Replace with your WiFi SSID
const char* password = "98989898";  // Replace with your WiFi password

// Web server and WebSocket
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);

// Function declarations
void updateDisplay();
void handleSerialCommands();
void handleWebSocket(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendWebSocketData();
int  encoderTargetTemp = 100;

// Initialize display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Initialize MAX6675
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Initialize Reflow Controller
ReflowController reflowController;

// Initialize ENCODER
RotaryEncoder encoder(ENCODER_CLK, ENCODER_DT);

// PID Controller configuration
double Setpoint, Input, Output;
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Temperature variables
double currentTemp = 0.0;
unsigned long lastTempRead = 0;

// Display update timing
unsigned long lastDisplayUpdate = 0;

// Cycle burst control variables
unsigned long lastCycleBurst = 0;
bool heaterState = false;

// System state
bool heatingEnabled = true;
bool alarmState = false;
const float hysteresis = 1.0;

void setup() {
        // Set initial encoder position
    encoder.setPosition(encoderTargetTemp);

// Initialize I2C for display 
Wire.begin(I2C_SDA, I2C_SCL);

// Initialize display
    Serial.println("Initializing Display...");
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed! Check wiring:"));
        for(;;);
    }
  display.clearDisplay();
display.setTextColor(SSD1306_WHITE);
display.setTextSize(1);
display.setCursor(0, 0);
//display.print(encoderTargetTemp);
//display.print(reflowController.getTargetTemperature(), 1);
display.display();
delay(1000);

    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    delay(STARTUP_DELAY);
    Serial.println("\nESP8266 Reflow Oven PID Controller Starting...");
    
    // Initialize LittleFS
    if(!LittleFS.begin()) {
        Serial.println("LittleFS initialization failed!");
        Serial.println("Please upload the file system image (PlatformIO: Upload Filesystem Image)");
        return;
    }
    Serial.println("File system initialized successfully!");
    
    // Debug: List files in file system
    Dir dir = LittleFS.openDir("/");
    Serial.println("Files in file system:");
    while (dir.next()) {
        Serial.print("  - ");
        Serial.print(dir.fileName());
        Serial.print(" (");
        Serial.print(dir.fileSize());
        Serial.println(" bytes)");
    }
    /*
    
    // Connect to WiFi
  WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("========================================");
    Serial.println("          Network Information           ");
    Serial.println("========================================");
    Serial.print("Connected to WiFi: ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("To access the interface:");
    Serial.print("1. Main dashboard: http://");
    Serial.println(WiFi.localIP());
    Serial.print("2. WebSocket URL: ws://");
    Serial.print(WiFi.localIP());
    Serial.println(":81");
    Serial.println("========================================");
    
    // Initialize web server
    server.on("/", HTTP_GET, []() {
        Serial.println("Web request received for /");
        if (LittleFS.exists("/index.html")) {
            Serial.println("Found index.html, serving...");
            File file = LittleFS.open("/index.html", "r");
            if (file) {
                Serial.printf("File size: %d bytes\n", file.size());
                server.streamFile(file, "text/html");
                file.close();
                Serial.println("File served successfully");
            } else {
                Serial.println("Error opening index.html!");
                server.send(500, "text/plain", "Internal Server Error");
            }
        } else {
            Serial.println("Error: index.html not found in file system!");
            // List all files to help debug
            Dir dir = LittleFS.openDir("/");
            Serial.println("Available files:");
            while (dir.next()) {
                Serial.printf("  - %s (%d bytes)\n", dir.fileName().c_str(), dir.fileSize());
            }
            server.send(404, "text/plain", "File not found");
        }
    });
    server.begin();
    
    // Initialize WebSocket
    webSocket.begin();
    webSocket.onEvent(handleWebSocket);
    // Initialize components
    reflowController.begin(); */
    
    // Initialize PID
    Setpoint = AMBIENT_TEMP;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTime(TEMP_READ_INTERVAL);
    
    // Initialize SCR control pin
    pinMode(SCR_CONTROL_PIN, OUTPUT);
    analogWrite(SCR_CONTROL_PIN, 0);
    
    Serial.println("System initialized successfully!");
}

void handleWebSocket(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected from url: %s\n", num, payload);
            break;
            
        case WStype_TEXT:
            const size_t bufferSize = 32;  // Adjust size as needed for expected payloads
            char command[bufferSize];
            strncpy(command, (char*)payload, bufferSize - 1);
            command[bufferSize - 1] = '\0';  // Ensure null termination
            
            if (strncmp(command, "start", bufferSize) == 0) {
                heatingEnabled = true;
                reflowController.start();
            } else if (strncmp(command, "stop", bufferSize) == 0) {
                heatingEnabled = false;
                reflowController.stop();
            }
            break;
    }
}

void sendWebSocketData() {
    StaticJsonDocument<512> doc;  // Increased size for profile data
    
    // Current status
    doc["temp"] = currentTemp;
    doc["setpoint"] = reflowController.getTargetTemperature();
    doc["state"] = ReflowController::getStageString(reflowController.getCurrentStage());
    doc["isRunning"] = reflowController.isRunning();
    doc["heaterOn"] = heaterState;
    
    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
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
    /*display.setCursor(0, 35);
    display.print("Target: ");
    display.print(reflowController.getTargetTemperature(), 1);
    display.print("C");*/
    
    // PID Output
    display.setCursor(0, 45);
    display.print("Power: ");
    display.print((Output/255.0)*100, 0);
    display.print("%");

    //  Add encoder value
    display.setCursor(0, 35);
    display.print("Enc: ");
    display.print(encoderTargetTemp);
    display.print("C");
    
    // Check if current temperature has reached or exceeded target
if (currentTemp >= encoderTargetTemp + hysteresis) {
    // Temperature is above target + buffer, turn off heater
    digitalWrite(SCR_CONTROL_PIN, LOW);
    heaterState = false;
    heatingEnabled = false;
    Serial.println("Heater OFF: Temperature exceeded target.");
}
else if (currentTemp <= encoderTargetTemp - hysteresis) {
    // Temperature dropped below target - buffer, turn heater ON again
    heatingEnabled = true;
    Serial.println("Heater ON: Temperature dropped below target.");
}
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

    // Send real-time data over Serial with current state
    Serial.print("State:");
    Serial.print(ReflowController::getStageString(reflowController.getCurrentStage()));
    Serial.print(",T:");
    Serial.print(currentTemp, 1);
    Serial.print(",S:");
    Serial.print(reflowController.getTargetTemperature(), 1);
    Serial.print(",P:");
    Serial.print((Output/255.0)*100, 1);
    if (alarmState) {
        Serial.print(",ALARM");
    } else if (!heatingEnabled) {
        Serial.print(",DISABLED");
    }
    Serial.println();
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

void loop() 
{
    // Handle web server
    server.handleClient();
    webSocket.loop();
    
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
    
    // PID calculation and Cycle Burst SCR control
    if (heatingEnabled && !alarmState) {
        myPID.Compute();
        
        // Calculate ON time based on PID output (0-255 mapped to 0-CYCLE_BURST_PERIOD)
        unsigned long onTime = (Output * CYCLE_BURST_PERIOD) / 255;
        
        // Ensure minimum ON/OFF times
        if (onTime < MIN_BURST_TIME) {
            onTime = 0;
        } else if (onTime > (CYCLE_BURST_PERIOD - MIN_BURST_TIME)) {
            onTime = CYCLE_BURST_PERIOD;
        }
        
        // Implement cycle burst control
        unsigned long currentTime = millis();
        unsigned long cycleTime = currentTime - lastCycleBurst;
        
        if (cycleTime >= CYCLE_BURST_PERIOD) {
            lastCycleBurst = currentTime;
            cycleTime = 0;
        }
        
        // Set heater state based on cycle time
        if (cycleTime < onTime) {
            if (!heaterState) {
                digitalWrite(SCR_CONTROL_PIN, HIGH);
                heaterState = true;
            }
        } else {
            if (heaterState) {
                digitalWrite(SCR_CONTROL_PIN, LOW);
                heaterState = false;
            }
        }
    } else {
        Output = 0;
        digitalWrite(SCR_CONTROL_PIN, LOW);
        heaterState = false;
    }
    encoder.tick(); 

int newTemp = encoder.getPosition();
if (newTemp != encoderTargetTemp) {
    encoderTargetTemp = newTemp;
    
    // Clamp value within safe limits
    if (encoderTargetTemp < 30) encoderTargetTemp = 30;
    if (encoderTargetTemp > 250) encoderTargetTemp = 250;

    // Set PID target temperature
    Setpoint = encoderTargetTemp;
    
    Serial.print("Target Temp set to: ");
    Serial.println(Setpoint);
}
    // Update display and WebSocket
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay();
        sendWebSocketData();
        lastDisplayUpdate = millis();
    }
    
    // Handle serial commands
    handleSerialCommands();
}

