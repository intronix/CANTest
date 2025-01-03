#include "eeprom.h"

extern I2C_HandleTypeDef hi2c1;  // I2C handle from main.c

void EEPROM_Init(void)
{
    // Add any initialization if needed
    HAL_Delay(10);  // Small delay for EEPROM to be ready after power-up
}

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t address, uint8_t data)
{
    HAL_StatusTypeDef status;
    
    // Write byte to EEPROM
    status = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, address, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    
    // Wait for write cycle to complete (typical write cycle time is 5ms)
    HAL_Delay(5);
    
    return status;
}

uint8_t EEPROM_ReadByte(uint16_t address)
{
    uint8_t data;
    
    // Read byte from EEPROM
    HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, address, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    
    return data;
}
