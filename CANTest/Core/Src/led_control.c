#include "led_control.h"
#include "encoder.h"

// Global variables
uint8_t buttonPressed = 0;
uint8_t longPressDetected = 0;
uint32_t pressStartTime = 0;
uint8_t ledState = 0;  // Start with LED off

void Check_Button(void)
{
    static uint8_t lastButtonState = GPIO_PIN_SET;
    uint8_t currentButtonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);

    // Debounce logic
    if (currentButtonState != lastButtonState)
    {
        HAL_Delay(DEBOUNCE_DELAY);
        currentButtonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
    }

    // Button just pressed
    if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
        buttonPressed = 1;
        longPressDetected = 0;
        pressStartTime = HAL_GetTick();
    }
    // Button just released
    else if (currentButtonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET)
    {
        if (!longPressDetected && buttonPressed)
        {
            Single_Click_Action();
        }
        buttonPressed = 0;
        longPressDetected = 0;
    }
    // Button is being held down
    else if (currentButtonState == GPIO_PIN_RESET && buttonPressed && !longPressDetected)
    {
        if (HAL_GetTick() - pressStartTime > LONG_PRESS_DELAY)
        {
            Long_Press_Action();
            longPressDetected = 1;
        }
    }

    // Update DAC value if LED is on
    if (ledState)
    {
        MCP4725_Write(Encoder_GetDACValue());
    }

    lastButtonState = currentButtonState;
}

void Single_Click_Action(void)
{
    // Only take action if LED is currently off
    if (!ledState)
    {
        // Set DAC to current encoder value
        MCP4725_Write(Encoder_GetDACValue());
        ledState = 1;
    }
}

void Long_Press_Action(void)
{
    // Only take action if LED is currently on
    if (ledState)
    {
    	// Set DAC to LED off value
        MCP4725_Write(LED_OFF_VALUE);
        ledState = 0;
    }
}
