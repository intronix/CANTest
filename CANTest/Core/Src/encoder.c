#include "encoder.h"
#include "dac.h"
#include "led_control.h"
#include "eeprom.h"
#include "buzzer.h"
#include <stdlib.h>

// Global variables
int16_t encoderPosition = 0;    // Current step (0-19)
uint16_t dacValue = DAC_MIN_VALUE;  // Current DAC value
uint16_t dacLowValue = DAC_MIN_VALUE;
uint16_t dacHighValue = DAC_MAX_VALUE;
EncoderMode_t encoderMode = NORMAL_MODE;

static TIM_HandleTypeDef *encoderTimer;
static uint16_t lastCount = 0;    // Last timer count value
static uint16_t lastPosition = 0;  // Last valid position
static int8_t accumulatedCounts = 0;  // Accumulated counts for step detection

// Buzzer control variables
static uint32_t lastBuzzTime = 0;
static uint8_t buzzCount = 0;
static uint8_t buzzState = 0;
static uint8_t buzzerActive = 0;

void Encoder_Init(TIM_HandleTypeDef *htim)
{
    encoderTimer = htim;
    HAL_TIM_Encoder_Start(encoderTimer, TIM_CHANNEL_ALL);
    
    // Initialize position and DAC value
    encoderPosition = 0;
    lastPosition = 0;
    
    // Load calibration values first
    Encoder_LoadCalibration();
    
    // Set initial DAC value to dacLowValue in normal mode
    dacValue = dacLowValue;
    accumulatedCounts = 0;
    
    // Initialize buzzer variables
    lastBuzzTime = 0;
    buzzCount = 0;
    buzzState = 0;
    buzzerActive = 0;
    
    // Reset counter
    __HAL_TIM_SET_COUNTER(encoderTimer, 0);
    lastCount = 0;
}

void Process_Buzzer(void)
{
    if (!buzzerActive) return;
    
    uint32_t currentTime = HAL_GetTick();
    
    // Check if it's time for next buzz action
    if (currentTime - lastBuzzTime >= 100)  // 100ms interval
    {
        if (encoderPosition == 0)  // Single ON-OFF for position 0
        {
            if (buzzCount == 0)
            {
                Buzzer_On();
                buzzCount++;
            }
            else if (buzzCount == 1)
            {
                Buzzer_Off();
                buzzCount = 0;
                buzzerActive = 0;  // Sequence complete
            }
        }
        else if (encoderPosition == ENCODER_STEPS - 1)  // ON-OFF-ON-OFF for max position
        {
            switch(buzzCount)
            {
                case 0:  // First ON
                    Buzzer_On();
                    break;
                case 1:  // First OFF
                    Buzzer_Off();
                    break;
                case 2:  // Second ON
                    Buzzer_On();
                    break;
                case 3:  // Second OFF
                    Buzzer_Off();
                    buzzerActive = 0;  // Sequence complete
                    buzzCount = 0;
                    return;
            }
            buzzCount++;
        }
        lastBuzzTime = currentTime;
    }
}

void Start_Buzz_Sequence(uint8_t position)
{
    if (!buzzerActive)  // Only start if not already buzzing
    {
        buzzerActive = 1;
        buzzCount = 0;
        buzzState = 0;
        lastBuzzTime = HAL_GetTick();
    }
}

void Encoder_Update(void)
{
    uint16_t counter = __HAL_TIM_GET_COUNTER(encoderTimer);
    
    // Process any active buzzer sequence
    Process_Buzzer();
    
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
                int16_t oldPosition = encoderPosition;
                encoderPosition = newPosition;
                lastPosition = newPosition;
                
                // Update DAC value based on mode
                if (encoderMode == NORMAL_MODE)
                {
                    // Map encoder position to calibrated range (dacLowValue to dacHighValue)
                    uint32_t range = dacHighValue - dacLowValue;
                    uint32_t step = (range * encoderPosition) / (ENCODER_STEPS - 1);
                    dacValue = dacLowValue + step;
                    
                    // Check if we've reached min or max position
                    if (encoderPosition == 0 || encoderPosition == ENCODER_STEPS - 1)
                    {
                        Start_Buzz_Sequence(encoderPosition);
                    }
                }
                else // Calibration modes
                {
                    // Direct mapping to full DAC range for calibration
                    dacValue = (DAC_MAX_VALUE * encoderPosition) / (ENCODER_STEPS - 1);
                }
                
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

void Encoder_LoadCalibration(void) {
    // Read calibration values from EEPROM
    dacLowValue = EEPROM_ReadWord(EEPROM_DAC_LOW_ADDR);
    dacHighValue = EEPROM_ReadWord(EEPROM_DAC_HIGH_ADDR);
    
    // Validate read values and ensure they're within bounds
    if (dacLowValue > DAC_MAX_VALUE) {
        dacLowValue = DAC_MIN_VALUE;
    }
    if (dacHighValue > DAC_MAX_VALUE) {
        dacHighValue = DAC_MAX_VALUE;
    }
    if (dacLowValue >= dacHighValue) {
        dacLowValue = DAC_MIN_VALUE;
        dacHighValue = DAC_MAX_VALUE;
    }
}

void Encoder_SaveCalibration(void) {
    // Ensure DAC value is within bounds before saving
    if (dacValue > DAC_MAX_VALUE) {
        dacValue = DAC_MAX_VALUE;
    }
    if (dacValue < DAC_MIN_VALUE) {
        dacValue = DAC_MIN_VALUE;
    }
    
    // Save current DAC value to appropriate calibration
    if (encoderMode == CALIBRATE_LOW) {
        dacLowValue = dacValue;
        EEPROM_WriteWord(EEPROM_DAC_LOW_ADDR, dacLowValue);
    } else if (encoderMode == CALIBRATE_HIGH) {
        dacHighValue = dacValue;
        EEPROM_WriteWord(EEPROM_DAC_HIGH_ADDR, dacHighValue);
    }
}

void Encoder_SetMode(EncoderMode_t mode) {
    encoderMode = mode;
    // Reset encoder position when changing modes
    __HAL_TIM_SET_COUNTER(encoderTimer, 0);
    encoderPosition = 0;
    lastPosition = 0;
    lastCount = 0;
    accumulatedCounts = 0;
    
    // Set initial DAC value based on mode
    if (mode == NORMAL_MODE) {
        dacValue = dacLowValue; // Start from low value in normal mode
    } else {
        dacValue = DAC_MIN_VALUE; // Start from min in calibration mode
    }
}
