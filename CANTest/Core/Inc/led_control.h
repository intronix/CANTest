#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "main.h"

// Button defines
#define BUTTON_PIN GPIO_PIN_10
#define BUTTON_PORT GPIOA
#define DEBOUNCE_DELAY 50    // 50 ms debounce
#define LONG_PRESS_DELAY 500 // 500 ms for a long press

// LED brightness values
#define LED_OFF_VALUE 0     // Value when LED is turned off

// Global variables
extern uint8_t outputLedState;

// Function prototypes
void Check_Button(void);
void Single_Click_Action(void);
void Long_Press_Action(void);

#endif /* LED_CONTROL_H */
