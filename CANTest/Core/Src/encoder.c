#include "encoder.h"
#include "dac.h"
#include "led_control.h"

// Global variables
int16_t encoderPosition = 0;    // Current step (0-19)
uint16_t dacValue = DAC_MIN_VALUE;  // Current DAC value
static uint16_t lastCount = 0;    // Last timer count value

static TIM_HandleTypeDef *encoderTimer;

void Encoder_Init(TIM_HandleTypeDef *htim)
{
    encoderTimer = htim;
    HAL_TIM_Encoder_Start(encoderTimer, TIM_CHANNEL_ALL);
    
    // Initialize position and DAC value
    encoderPosition = 0;
    dacValue = DAC_MIN_VALUE;
    
    // Reset counter
    __HAL_TIM_SET_COUNTER(encoderTimer, 0);
    lastCount = 0;
}

void Encoder_Update(void)
{
    uint16_t counter = __HAL_TIM_GET_COUNTER(encoderTimer);
    
    // Check for counter change
    if (counter != lastCount)
    {
        // Determine rotation direction
        int8_t direction;
        
        // Handle counter overflow/underflow
        if (counter > lastCount)
        {
            if (counter - lastCount > 32768) // Overflow, actually went backwards
                direction = -1;
            else
                direction = 1;
        }
        else
        {
            if (lastCount - counter > 32768) // Underflow, actually went forwards
                direction = 1;
            else
                direction = -1;
        }
        
        // Check if we're at limits and trying to go the wrong way
        if ((encoderPosition == 0 && direction < 0) || 
            (encoderPosition >= ENCODER_STEPS-1 && direction > 0))
        {
            // Reset counter to current position
            __HAL_TIM_SET_COUNTER(encoderTimer, encoderPosition * ENCODER_COUNTS_PER_STEP);
            lastCount = encoderPosition * ENCODER_COUNTS_PER_STEP;
            return;
        }
        
        // Calculate new position
        int16_t newPosition = encoderPosition + direction;
        
        // Update position if within bounds
        if (newPosition >= 0 && newPosition < ENCODER_STEPS)
        {
            encoderPosition = newPosition;
            
            // Calculate new DAC value
            dacValue = DAC_MIN_VALUE + (encoderPosition * DAC_STEP_SIZE);
            
            // Ensure DAC value is within bounds
            if (dacValue > DAC_MAX_VALUE)
                dacValue = DAC_MAX_VALUE;
            else if (dacValue < DAC_MIN_VALUE)
                dacValue = DAC_MIN_VALUE;
        }
        
        // Update last count
        lastCount = __HAL_TIM_GET_COUNTER(encoderTimer);
    }
}

uint16_t Encoder_GetDACValue(void)
{
    return dacValue;
}
