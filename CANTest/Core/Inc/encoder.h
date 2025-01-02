#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

// Encoder limits for 12-bit DAC
#define ENCODER_MAX_VALUE 4094  // Maximum DAC value (12-bit - 1)
#define ENCODER_MIN_VALUE 0     // Minimum DAC value

// Button defines
#define BUTTON_PIN GPIO_PIN_10
#define BUTTON_PORT GPIOA
#define DEBOUNCE_DELAY 50    // 50 ms debounce
#define LONG_PRESS_DELAY 500 // 500 ms for a long press

// Global variables to be accessed from main
extern int16_t encoderPosition;
extern int16_t lastPosition;
extern uint8_t buttonPressed;
extern uint8_t longPressDetected;
extern uint32_t pressStartTime;

// Function prototypes
void Encoder_Init(TIM_HandleTypeDef *htim);
void Encoder_Update(void);
void Check_Button(void);
void Single_Click_Action(void);
void Long_Press_Action(void);

#endif /* ENCODER_H */
