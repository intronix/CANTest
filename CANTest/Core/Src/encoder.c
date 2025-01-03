#include "encoder.h"
#include "dac.h"
#include "led_control.h"

// Global variables
int16_t encoderPosition = 0;    // Current step (0-19)
uint16_t dacValue = DAC_MIN_VALUE;  // Current DAC value
static uint16_t lastCount = 0;    // Last timer count value
static uint16_t lastPosition = 0;  // Last valid position
static int8_t accumulatedCounts = 0;  // Accumulated counts for step detection

static TIM_HandleTypeDef *encoderTimer;

void Encoder_Init(TIM_HandleTypeDef *htim)
{
    encoderTimer = htim;
    HAL_TIM_Encoder_Start(encoderTimer, TIM_CHANNEL_ALL);
    
    // Initialize position and DAC value
    encoderPosition = 0;
    lastPosition = 0;
    dacValue = DAC_MIN_VALUE;
    accumulatedCounts = 0;
    
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
        int16_t delta;
        
        // Calculate delta considering counter wrap-around
        if (counter > lastCount)
        {
            if (counter - lastCount > 32768)  // Negative direction
                delta = -1;
            else                              // Positive direction
                delta = 1;
        }
        else
        {
            if (lastCount - counter > 32768)  // Positive direction
                delta = 1;
            else                              // Negative direction
                delta = -1;
        }
        
        // Accumulate counts
        accumulatedCounts += delta;
        
        // Check if we've accumulated enough counts for a step
        if (abs(accumulatedCounts) >= 4)
        {
            // Calculate direction from accumulated counts
            int8_t stepDirection = (accumulatedCounts > 0) ? 1 : -1;
            
            // Calculate potential new position
            int16_t newPosition = encoderPosition + stepDirection;
            
            // Only update if within bounds
            if (newPosition >= 0 && newPosition < ENCODER_STEPS)
            {
                encoderPosition = newPosition;
                lastPosition = newPosition;
                
                // Calculate new DAC value
                dacValue = DAC_MIN_VALUE + (encoderPosition * DAC_STEP_SIZE);
                
                // Ensure DAC value is within bounds
                if (dacValue > DAC_MAX_VALUE)
                    dacValue = DAC_MAX_VALUE;
                else if (dacValue < DAC_MIN_VALUE)
                    dacValue = DAC_MIN_VALUE;
            }
            else
            {
                // Reset counter to last valid position
                uint16_t resetCount = lastPosition * ENCODER_COUNTS_PER_STEP;
                __HAL_TIM_SET_COUNTER(encoderTimer, resetCount);
                counter = resetCount;
            }
            
            // Reset accumulated counts
            accumulatedCounts = 0;
        }
        
        // Update last count
        lastCount = counter;
    }
}

uint16_t Encoder_GetDACValue(void)
{
    return dacValue;
}
