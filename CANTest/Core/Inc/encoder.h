#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

// Encoder physical properties
#define ENCODER_STEPS 20        // Number of physical steps per rotation
#define ENCODER_COUNTS_PER_STEP 1  // Timer counts per encoder step
#define ENCODER_FULL_ROTATION (ENCODER_STEPS * ENCODER_COUNTS_PER_STEP)  // Total counts for full rotation

// DAC value range
#define DAC_MIN_VALUE 600   // Minimum DAC value for LED
#define DAC_MAX_VALUE 3160  // Maximum DAC value for LED

// Calculate step size for DAC values
#define DAC_STEP_SIZE ((DAC_MAX_VALUE - DAC_MIN_VALUE) / ENCODER_STEPS)

// Global variables to be accessed from main
extern int16_t encoderPosition;  // Current step (0-19)
extern uint16_t dacValue;       // Current DAC value (600-3160)

// Function prototypes
void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update(void);
uint16_t Encoder_GetDACValue(void);

#endif /* ENCODER_H */
