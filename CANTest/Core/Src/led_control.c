#include "led_control.h"
#include "dac.h"
#include "encoder.h"

// Global variables
uint8_t outputLedState = 0;  // Renamed from ledState
uint32_t pressStartTime = 0;
uint8_t buttonPressed = 0;
uint8_t longPressDetected = 0;

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
    if (outputLedState)
    {
        MCP4725_Write(Encoder_GetDACValue());
    }

    lastButtonState = currentButtonState;
}

void Single_Click_Action(void)
{
    outputLedState = 1;
   
    MCP4725_Write(dacLowValue);
    if(encoderPosition == 0)
        encoderPosition = 1;
}

void Long_Press_Action(void)
{
    MCP4725_Write(LED_OFF_VALUE);
    outputLedState = 0;
}
