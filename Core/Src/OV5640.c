/*
 * OV_5640.C
 *
 *  Created on: May 11, 2025
 *      Author: zac23
 */

//Standard Header Files
#include "i2c.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "dcmi.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"
//Project Header Files
#include "OV5640.h"
//Project Defines
#define OV5640_I2C_ADDR (0x78)
#define OV5640_RES_WIDTH 240
#define OV5640_RES_HEIGHT 360
#define OV5640_PIXEL_FORMAT 0x61//0x6 is rgb565 0x1 is R first

//Project consts

const uint16_t MAX_UART_DELAY = 1000;

//Resolution config table
static const struct {
	uint16_t width;
	uint16_t height;

}OV5640_Resolutions[] = {
		[OV5640_RES_240X360] = {240, 360},
		[OV5640_RES_480X640] = {480, 640},
		[OV5640_RES_720X1080] = {720, 1080}
};
//Format table
static const OV5640_Formats[] = {
		[OV5640_FORMAT_RGB565] = 0x61,
		[OV5640_FORMAT_YUV422] = 0X1F,
		[OV5640_FORMAT_JPEG] = 0X01,
		[OV5640_FORMAT_RAW] = 0X03//rg/gb
};



HAL_StatusTypeDef OV5640_WriteReg(uint16_t regAddr, uint8_t data) {
	uint8_t transmitData[3];
	transmitData[0] = (regAddr >> 8);
	transmitData[1] = regAddr & 0xFF;
	transmitData[2] = data;

	return HAL_I2C_Master_Transmit(&hi2c1, OV5640_I2C_ADDR, transmitData, 3,
			HAL_MAX_DELAY);

}

HAL_StatusTypeDef OV5640_ReadReg(uint16_t regAddr, uint8_t *dataPtr) {
	char buff[25] = "readStart\r\n";
	HAL_UART_Transmit(&huart3, &buff, sizeof(buff), HAL_MAX_DELAY);
	uint8_t transmitData[2];
	transmitData[0] = (regAddr >> 8);
	transmitData[1] = regAddr & 0xFF;
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, OV5640_I2C_ADDR,
			transmitData, 2, HAL_MAX_DELAY);
	char buffStatus[50];
	sprintf(buffStatus, "transmit Status: %d \r\n", status);
	HAL_UART_Transmit(&huart3, buffStatus, strlen(buffStatus), HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	}
	return HAL_I2C_Master_Receive(&hi2c1, OV5640_I2C_ADDR, dataPtr, 1,
			HAL_MAX_DELAY);
}

HAL_StatusTypeDef OV5640_TestConnection(void) {

	uint8_t pid;
	HAL_StatusTypeDef status = OV5640_ReadReg(0x300A, &pid);

	if (status != HAL_OK) {
		return status;
	}

	if (pid != 0x56) {
		return HAL_ERROR;
	}

	return HAL_OK;
}
void OV5640_PowerUpSequence(void) {
	HAL_Delay(10);
	HAL_GPIO_WritePin(CAMERA_PWDN_GPIO_Port, CAMERA_PWDN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(CAMERA_RST_GPIO_Port, CAMERA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_Delay(5);

}
HAL_StatusTypeDef OV5640_SetResolution(OV5640_Resolution selectedResolution){
	HAL_StatusTypeDef status = HAL_OK;
	const uint16_t selectedWidth = OV5640_Resolutions[selectedResolution].width;
	const uint16_t selectedHeight = OV5640_Resolutions[selectedResolution].height;
	status | OV5640_WriteReg(OV5640_REG_OUT_WIDTH_H, selectedWidth>>8);
	status | OV5640_WriteReg(OV5640_REG_OUT_WIDTH_L, selectedWidth&0xFF);
	status | OV5640_WriteReg(OV5640_REG_OUT_HEIGHT_H, selectedHeight>>8);
	status | OV5640_WriteReg(OV5640_REG_OUT_HEIGHT_L, selectedHeight&0xFF);
	return status;
}
HAL_StatusTypeDef OV5640_SetFormat(OV5640_Format selectedFormat){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t formatData= OV5640_Formats[selectedFormat];
	status | OV5640_WriteReg(OV5640_REG_FORMAT_CTRL, formatData);
	return status;


}
void OV5640_ConfigureCamera(void){
	HAL_StatusTypeDef status = HAL_OK;
	if (OV5640_TestConnection != HAL_OK){
		return;
	}

	status | OV5640_SetResolution(OV5640_RES_240X360);
	status | OV5640_SetFormat(OV5640_FORMAT_RGB565);
	if (status!= HAL_OK){
		return;
	}

	HAL_GPIO_WritePin(I2C_SUCCESS_GPIO_Port, I2C_SUCCESS_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(I2C_SUCCESS_GPIO_Port, I2C_SUCCESS_Pin,
			GPIO_PIN_RESET);
	HAL_Delay(500);
	return;



}
