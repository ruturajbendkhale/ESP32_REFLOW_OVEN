#ifndef PROFILE_H
#define PROFILE_H

// Preheat Stage Parameters
#define PREHEAT_RAMP_RATE 2.0        // °C/s - Rate of temperature increase from ambient
#define SOAK_TARGET_TEMP 150.0       // °C - Temperature at which soak begins

// Soak Stage Parameters
#define SOAK_TEMP 165.0              // °C - Stable temperature for soak
#define SOAK_DURATION 60             // seconds - Time to hold at soak temperature

// Reflow Stage Parameters
#define REFLOW_RAMP_RATE 2.0         // °C/s - Rate from soak to peak
#define PEAK_TEMP 245.0              // °C - Maximum temperature for lead-free solder
#define LIQUIDUS_TEMP 217.0          // °C - Melting point of SAC305 lead-free solder
#define TIME_ABOVE_LIQUIDUS 60       // seconds - Time above melting point

// Cooling Stage Parameters
#define COOLING_RATE 3.0             // °C/s - Rate of cooling
#define SAFE_HANDLING_TEMP 100.0     // °C - Temperature safe for handling

// Profile Timing Parameters
#define AMBIENT_TEMP 25.0            // °C - Assumed starting room temperature
#define PROFILE_CHECK_INTERVAL 1000   // ms - How often to update target temperature

// Profile Stage Definitions
enum ReflowStage {
    STAGE_IDLE,
    STAGE_PREHEAT,
    STAGE_SOAK,
    STAGE_REFLOW,
    STAGE_COOLING,
    STAGE_COMPLETE
};

// Profile Status Structure
struct ReflowStatus {
    ReflowStage currentStage;
    unsigned long stageStartTime;
    double targetTemp;
    double elapsedTime;
    bool isComplete;
};

#endif // PROFILE_H 