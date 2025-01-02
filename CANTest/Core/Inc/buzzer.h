#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

// Buzzer pin definitions
#define BUZZER_PIN GPIO_PIN_0
#define BUZZER_PORT GPIOB

// Function prototypes
void Buzzer_Init(void);
void Buzzer_Toggle(void);
void Buzzer_On(void);
void Buzzer_Off(void);

#endif /* BUZZER_H */
