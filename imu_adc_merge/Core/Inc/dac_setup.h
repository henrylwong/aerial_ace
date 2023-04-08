#ifndef DAC_SETUP_H
#define DAC_SETUP_H

#include "stm32f3xx_hal.h"
#include "mcp4728.h"

HAL_StatusTypeDef MCP4728_Write_VRef_Select(I2C_HandleTypeDef *I2CHandler, dacChannelConfig config);
HAL_StatusTypeDef MCP4728_Write_PWRDWN_Select(I2C_HandleTypeDef *I2CHandler, uint8_t command);
void MCP4728_Write_GeneralCall(I2C_HandleTypeDef *I2CHandler, uint8_t command);
void MCP4728_Write_AllChannels_Same(I2C_HandleTypeDef *I2CHandler, uint16_t output);
void MCP4728_Write_AllChannels_Diff(I2C_HandleTypeDef *I2CHandler, dacChannelConfig output);
void MCP4728_Write_SingleChannel(I2C_HandleTypeDef *I2CHandler, uint8_t channel, uint16_t output);
void MCP4728_Init(I2C_HandleTypeDef *I2CHandler, dacChannelConfig output);

#endif
