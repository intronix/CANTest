#include "buzzer.h"

void Buzzer_Init(void)
{
    // Ensure buzzer is off initially
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void Buzzer_Toggle(void)
{
    HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}
