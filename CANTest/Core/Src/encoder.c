#include "encoder.h"
#include "dac.h"

// Global variables
int16_t encoderPosition = 0;
int16_t lastPosition = 0;
uint8_t buttonPressed = 0;
uint8_t longPressDetected = 0;
uint32_t pressStartTime = 0;

static TIM_HandleTypeDef *encoderTimer;

void Encoder_Init(TIM_HandleTypeDef *htim)
{
    encoderTimer = htim;
    HAL_TIM_Encoder_Start(encoderTimer, TIM_CHANNEL_ALL);
    encoderPosition = 0;
    lastPosition = 0;
}

void Encoder_Update(void)
{
    // Read current encoder position
    encoderPosition = (int16_t)((int16_t)(__HAL_TIM_GET_COUNTER(encoderTimer))/4);
    
    // Limit checking
    if(encoderPosition > ENCODER_MAX_VALUE)
    {
        __HAL_TIM_SET_COUNTER(encoderTimer, (uint16_t)ENCODER_MAX_VALUE*4);
        encoderPosition = ENCODER_MAX_VALUE;
    }
    else if(encoderPosition < ENCODER_MIN_VALUE)
    {
        __HAL_TIM_SET_COUNTER(encoderTimer, (uint16_t)ENCODER_MIN_VALUE);
        encoderPosition = ENCODER_MIN_VALUE;
    }

    // Update DAC if position changed
    if (encoderPosition != lastPosition)
    {
        MCP4725_Write(encoderPosition);
        lastPosition = encoderPosition;
    }
}

void Check_Button(void)
{
    static uint8_t lastButtonState = GPIO_PIN_SET; // Last known button state
    uint8_t currentButtonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);

    // Debounce logic
    if (currentButtonState != lastButtonState)
    {
        HAL_Delay(DEBOUNCE_DELAY);
        currentButtonState = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
    }

    if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
        // Button pressed
        buttonPressed = 1;
        longPressDetected = 0;
        pressStartTime = HAL_GetTick();
    }
    else if (currentButtonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET)
    {
        // Button released
        if (!longPressDetected && buttonPressed)
        {
            Single_Click_Action();
        }
        buttonPressed = 0;
    }
    else if (buttonPressed && !longPressDetected)
    {
        // Check for long press
        if (HAL_GetTick() - pressStartTime > LONG_PRESS_DELAY)
        {
            Long_Press_Action();
            longPressDetected = 1;
        }
    }

    lastButtonState = currentButtonState;
}

void Single_Click_Action(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void Long_Press_Action(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
