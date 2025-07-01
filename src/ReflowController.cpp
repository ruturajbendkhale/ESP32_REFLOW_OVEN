#include "ReflowController.h"

ReflowController::ReflowController()
    : reflowInProgress(false)
    , lastProfileUpdate(0)
{
    reflowStatus = {STAGE_IDLE, 0, AMBIENT_TEMP, 0, false};
}

void ReflowController::begin() {
    stop();  // Reset to initial state
}

void ReflowController::start() {
    reflowInProgress = true;
    reflowStatus = {STAGE_IDLE, millis(), AMBIENT_TEMP, 0, false};
    Serial.println("Starting reflow process");
}

void ReflowController::stop() {
    reflowInProgress = false;
    reflowStatus.currentStage = STAGE_IDLE;
    reflowStatus.isComplete = false;
    Serial.println("Reflow process stopped");
}

void ReflowController::update(double currentTemp) {
    if (!reflowInProgress) return;
    
    if (millis() - lastProfileUpdate >= PROFILE_CHECK_INTERVAL) {
        updateStage(currentTemp);
        reflowStatus.targetTemp = calculateTargetTemperature();
        lastProfileUpdate = millis();
    }
}

double ReflowController::calculateTargetTemperature() {
    double elapsedTimeInStage = (millis() - reflowStatus.stageStartTime) / 1000.0; // Convert to seconds
    
    switch(reflowStatus.currentStage) {
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

void ReflowController::updateStage(double currentTemp) {
    unsigned long currentTime = millis();
    double elapsedTimeInStage = (currentTime - reflowStatus.stageStartTime) / 1000.0; // seconds
    
    switch(reflowStatus.currentStage) {
        case STAGE_IDLE:
            reflowStatus.currentStage = STAGE_PREHEAT;
            reflowStatus.stageStartTime = currentTime;
            Serial.println("Starting Preheat Stage");
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
                Serial.println("Reflow Process Complete");
            }
            break;
            
        case STAGE_COMPLETE:
            // Do nothing, wait for manual reset
            break;
    }
    
    // Update elapsed time
    reflowStatus.elapsedTime = (currentTime - reflowStatus.stageStartTime) / 1000.0;
}

const char* ReflowController::getStageString(ReflowStage stage) {
    switch(stage) {
        case STAGE_IDLE: return "IDLE";
        case STAGE_PREHEAT: return "PREHEAT";
        case STAGE_SOAK: return "SOAK";
        case STAGE_REFLOW: return "REFLOW";
        case STAGE_COOLING: return "COOLING";
        case STAGE_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
} 