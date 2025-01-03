#include "eeprom.h"

extern I2C_HandleTypeDef hi2c1;  // I2C handle from main.c

void EEPROM_Init(void)
{
    // Add any initialization if needed
    HAL_Delay(10);  // Small delay for EEPROM to be ready after power-up
}

HAL_StatusTypeDef EEPROM_WriteWord(uint16_t address, uint16_t data)
{
    uint8_t buffer[3];
    HAL_StatusTypeDef status;
    
    // Write low byte
    buffer[0] = address & 0xFF;  // Address byte
    buffer[1] = data & 0xFF;     // Low byte
    
    status = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    
    HAL_Delay(5); // Wait for write to complete
    
    // Write high byte
    buffer[0] = (address + 1) & 0xFF;  // Address byte
    buffer[1] = (data >> 8) & 0xFF;    // High byte
    
    status = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    
    HAL_Delay(5); // Wait for write to complete
    
    return HAL_OK;
}

uint16_t EEPROM_ReadWord(uint16_t address)
{
    uint8_t buffer[2];
    uint16_t data = 0;
    
    // Read low byte
    buffer[0] = address & 0xFF;  // Address byte
    HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR, &buffer[1], 1, HAL_MAX_DELAY);
    data = buffer[1];
    
    HAL_Delay(1); // Small delay between reads
    
    // Read high byte
    buffer[0] = (address + 1) & 0xFF;  // Address byte
    HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR, &buffer[1], 1, HAL_MAX_DELAY);
    data |= (uint16_t)buffer[1] << 8;
    
    return data;
}

// These functions are not used but kept for compatibility
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t address, uint8_t data)
{
    uint8_t buffer[2];
    buffer[0] = address & 0xFF;  // Address byte
    buffer[1] = data;           // Data byte
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    
    HAL_Delay(5); // Wait for write to complete
    return HAL_OK;
}

uint8_t EEPROM_ReadByte(uint16_t address)
{
    uint8_t buffer[2];
    
    buffer[0] = address & 0xFF;  // Address byte
    HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, EEPROM_ADDR, &buffer[1], 1, HAL_MAX_DELAY);
    
    return buffer[1];
}
