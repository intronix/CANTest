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
static uint8_t isCalibrationBuzz = 0;  // Flag for calibration buzz pattern
static uint8_t buzzOff = 0;  // Flag for buzz off state

// External variables
extern uint8_t outputLedState;  // LED state from led_control.c

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
    isCalibrationBuzz = 0;
    buzzOff = 0;
    
    // Reset counter
    __HAL_TIM_SET_COUNTER(encoderTimer, 0);
    lastCount = 0;
}

static void Start_Calibration_Buzz(void)
{
    if (!buzzerActive)
    {
        buzzerActive = 1;
        isCalibrationBuzz = 1;
        Buzzer_On();
        lastBuzzTime = HAL_GetTick();
    }
}

static void Process_Buzzer(void)
{
    uint32_t currentTime = HAL_GetTick();
    
    if (buzzerActive)
    {
        // Handle calibration buzz
        if (isCalibrationBuzz)
        {
            if (currentTime - lastBuzzTime >= 1000)  // 1 second buzz
            {
                Buzzer_Off();
                buzzerActive = 0;
                isCalibrationBuzz = 0;
            }
            return;
        }

        // If buzzer has been on too long (safety timeout)
        if (currentTime - lastBuzzTime > 2000)  // 2 second safety timeout
        {
            Buzzer_Off();
            buzzerActive = 0;
            buzzCount = 0;
            return;
        }

        // Handle double buzz sequence
        if (buzzCount > 0)
        {
            if (currentTime - lastBuzzTime >= BUZZ_ON_TIME && !buzzOff)
            {
                Buzzer_Off();
                lastBuzzTime = currentTime;
                buzzOff = 1;
            }
            else if (currentTime - lastBuzzTime >= BUZZ_OFF_TIME && buzzOff)
            {
                if (buzzCount > 1)
                {
                    Buzzer_On();
                    lastBuzzTime = currentTime;
                    buzzCount--;
                    buzzOff = 0;
                }
                else
                {
                    buzzerActive = 0;
                    buzzCount = 0;
                }
            }
        }
    }
}

static void Start_Buzz_Sequence(int16_t position)
{
    static uint32_t lastBuzzTriggerTime = 0;
    uint32_t currentTime = HAL_GetTick();
    
    // Only proceed if LED is on in normal mode, or always proceed in calibration mode
    if (encoderMode == NORMAL_MODE && !outputLedState)
    {
        return;
    }
    
    // Debounce buzzer triggers - prevent retriggering within 500ms
    if (currentTime - lastBuzzTriggerTime < 500)
    {
        return;
    }
    
    // Only start new sequence if buzzer is not active
    if (!buzzerActive)
    {
        buzzerActive = 1;
        buzzOff = 0;
        lastBuzzTime = currentTime;
        lastBuzzTriggerTime = currentTime;
        
        if (position == 0)
        {
            // Single buzz for position 0
            Buzzer_On();
            buzzCount = 1;
        }
        else if (position == ENCODER_STEPS - 1)
        {
            // Double buzz for max position
            Buzzer_On();
            buzzCount = 2;
        }
    }
}

void Encoder_Update(void)
{
    uint16_t counter = __HAL_TIM_GET_COUNTER(encoderTimer);
    static uint16_t lastDacValue = 0;
    
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
                encoderPosition = newPosition;
                lastPosition = newPosition;
                
                // Update DAC value based on mode
                if (encoderMode == NORMAL_MODE)
                {
                    if (encoderPosition == 0)
                    {
                        // Force DAC to 0 at position 0
                        dacValue = 0;
                    }
                    else if (encoderPosition == 1)
                    {
                        // Force DAC to dacLowValue at position 1
                        dacValue = dacLowValue;
                    }
                    else
                    {
                        // Map encoder position (2 to 19) to calibrated range (dacLowValue to dacHighValue)
                        uint32_t range = dacHighValue - dacLowValue;
                        uint32_t step = (range * (encoderPosition - 1)) / (ENCODER_STEPS - 2);
                        dacValue = dacLowValue + step;
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
                
                // Check if we've reached min or max position for buzzer feedback
                if (encoderPosition == 0 || encoderPosition == ENCODER_STEPS - 1)
                {
                    Start_Buzz_Sequence(encoderPosition);
                }
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
    
    // Check if we're at position 1 and DAC value needs to be corrected
    if (encoderPosition == 1 && dacValue != dacLowValue)
    {
        dacValue = dacLowValue;
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
        Start_Calibration_Buzz();  // Long buzz for calibration set
    } else if (encoderMode == CALIBRATE_HIGH) {
        dacHighValue = dacValue;
        EEPROM_WriteWord(EEPROM_DAC_HIGH_ADDR, dacHighValue);
        Start_Calibration_Buzz();  // Long buzz for calibration set
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
