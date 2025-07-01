#ifndef REFLOW_CONTROLLER_H
#define REFLOW_CONTROLLER_H

#include <Arduino.h>
#include "PROFILE.h"

class ReflowController {
public:
    ReflowController();
    
    // Control methods
    void begin();
    void start();
    void stop();
    void update(double currentTemp);
    
    // Status methods
    bool isRunning() const { return reflowInProgress; }
    bool isComplete() const { return reflowStatus.isComplete; }
    ReflowStage getCurrentStage() const { return reflowStatus.currentStage; }
    double getTargetTemperature() const { return reflowStatus.targetTemp; }
    double getElapsedTime() const { return reflowStatus.elapsedTime; }
    
    // Status string helper
    static const char* getStageString(ReflowStage stage);
    
private:
    // Internal methods
    double calculateTargetTemperature();
    void updateStage(double currentTemp);
    
    // State variables
    bool reflowInProgress;
    ReflowStatus reflowStatus;
    unsigned long lastProfileUpdate;
};

#endif // REFLOW_CONTROLLER_H 