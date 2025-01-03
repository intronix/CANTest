#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

// Encoder physical properties
#define ENCODER_STEPS 20        // Number of physical steps per rotation
#define ENCODER_COUNTS_PER_STEP 1  // Timer counts per encoder step
#define ENCODER_FULL_ROTATION (ENCODER_STEPS * ENCODER_COUNTS_PER_STEP)  // Total counts for full rotation

// DAC value range
#define DAC_MIN_VALUE 0      // Absolute minimum DAC value
#define DAC_MAX_VALUE 4095   // Absolute maximum DAC value
#define DAC_STEP_SIZE ((DAC_MAX_VALUE - DAC_MIN_VALUE) / (ENCODER_STEPS - 1))  // Step size for normal mode

// Calibration mode states
typedef enum {
    NORMAL_MODE,
    CALIBRATE_LOW,
    CALIBRATE_HIGH
} EncoderMode_t;

// Global variables to be accessed from main
extern int16_t encoderPosition;    // Current step (0-19)
extern uint16_t dacValue;         // Current DAC value
extern uint16_t dacLowValue;      // Calibrated minimum DAC value
extern uint16_t dacHighValue;     // Calibrated maximum DAC value
extern EncoderMode_t encoderMode; // Current encoder operation mode

// Function prototypes
void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update(void);
uint16_t Encoder_GetDACValue(void);
void Encoder_LoadCalibration(void);
void Encoder_SaveCalibration(void);
void Encoder_SetMode(EncoderMode_t mode);

#endif /* ENCODER_H */
