#include <Arduino.h>
#include "config.h"
#include "init.h"
#include "globals.h"
#include "path_planing.h"

// ==========================
// Global Variables & Buffers
// ==========================

// Stores button state and press timing information
ButtonInfo currentButtonInfo = {0, 0, 0};

// Array to hold all steps used in path planning (maximum TOTAL_INDEX steps)
dataIndex ramIndexData[TOTAL_INDEX];

// List of PID controller parameter sets (up to NUM_PID profiles)
dataPID listPID[NUM_PID];

// System-wide configuration values (thresholds, speed, PID index, etc.)
dataSetting setting;

// Default threshold values for each of the 16 analog sensors (for testing/calibration)
unsigned int preDefinedThreshold[16] = {
  500, 500, 500, 500, 500, 500, 500, 500,
  500, 500, 500, 500, 500, 500, 500, 500
};

// Flag indicating whether sensor calibration has already been completed
bool previouslyCalibrated = true;

// Buffer for real-time analog readings from 16 sensors
uint16_t reading[16];
