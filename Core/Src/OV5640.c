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
#include "stm32h7xx_hal_dcmi.h"
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
		[OV5640_RES_240X320] = {240, 320},
		[OV5640_RES_480X640] = {480, 640},
		[OV5640_RES_720X1080] = {720, 1080}
};
//Format table
static const struct {
	uint16_t formatValue;
	uint16_t bytesPerPixel;

}OV5640_Formats[] = {
		[OV5640_FORMAT_RGB565] = {0x61,2},
		[OV5640_FORMAT_YUV422] = {0X1F,2},
		[OV5640_FORMAT_JPEG] = {0X01,1},
		[OV5640_FORMAT_RAW] = {0X03,2}//rg/gb
};

OV5640_CameraConfig activeCameraConfig = {
		OV5640_RES_240X320,
		OV5640_FORMAT_RGB565
};

OV5640_CameraConfig* activeCameraConfigPtr = &activeCameraConfig;

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
HAL_StatusTypeDef OV5640_PowerUpSequence(void) {
	HAL_Delay(10);
	HAL_GPIO_WritePin(CAMERA_PWDN_GPIO_Port, CAMERA_PWDN_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(CAMERA_RST_GPIO_Port, CAMERA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_Delay(5);
	HAL_StatusTypeDef status = OV5640_ConfigureCamera();
	return status;

}
HAL_StatusTypeDef OV5640_SetResolution(void){
	HAL_StatusTypeDef status = HAL_OK;
	const uint16_t selectedWidth =OV5640_Resolutions[activeCameraConfigPtr->resolution].width;
	const uint16_t selectedHeight = OV5640_Resolutions[activeCameraConfigPtr->resolution].height;
	status |= OV5640_WriteReg(OV5640_REG_OUT_WIDTH_H, selectedWidth>>8);
	status |= OV5640_WriteReg(OV5640_REG_OUT_WIDTH_L, selectedWidth&0xFF);
	status |= OV5640_WriteReg(OV5640_REG_OUT_HEIGHT_H, selectedHeight>>8);
	status |= OV5640_WriteReg(OV5640_REG_OUT_HEIGHT_L, selectedHeight&0xFF);
	return status;
}
HAL_StatusTypeDef OV5640_SetFormat(void){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t formatData= OV5640_Formats[activeCameraConfigPtr->pixelFormat].formatValue;
	status | OV5640_WriteReg(OV5640_REG_FORMAT_CTRL, formatData);
	return status;


}
HAL_StatusTypeDef OV5640_ConfigureCamera(void){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t resetValue = 0x0;

	OV5640_ReadReg(OV5640_SOFTWARE_RESET, &resetValue);
	resetValue |=0x80;
	OV5640_WriteReg(OV5640_SOFTWARE_RESET, resetValue);
	HAL_Delay(10);
	resetValue &= ~0x80;
	OV5640_WriteReg(OV5640_SOFTWARE_RESET, resetValue);
	HAL_Delay(100);

	status |= OV5640_SetResolution();
	status |= OV5640_SetFormat();
	uint8_t result;
	//write format for format control register to make format rgb565

	OV5640_ReadReg(OV5640_REG_FORMAT_CTRL, &result);
	char buff3[20];
	sprintf(buff3, "result of format: %d\n\r", result);
	HAL_UART_Transmit(&huart3, buff3, strlen(buff3), HAL_MAX_DELAY);
	return status;



}
void frameCapture(void){

	uint16_t width = OV5640_Resolutions[activeCameraConfigPtr->resolution].width;
	uint16_t height = OV5640_Resolutions[activeCameraConfigPtr->resolution].height;
	uint8_t pixelSize = OV5640_Formats[activeCameraConfigPtr->pixelFormat].bytesPerPixel;

	uint8_t frame[width*height*pixelSize];
    uint32_t timeout = HAL_MAX_DELAY;
    char testBuff[10] = "In Frame\n\r";
    HAL_UART_Transmit(&huart3, testBuff, strlen(testBuff), HAL_MAX_DELAY);
	DCMI->CR |=DCMI_CR_ENABLE;
	DCMI->CR |=DCMI_CR_CM;
	DCMI->CR |= DCMI_CR_CAPTURE;
	if (DCMI->RISR & 0x01<<0){
		uint32_t pixel = DCMI->DR;
		char buffFrame[20];
		sprintf(buffFrame, "Start of image: %d,\n\r", pixel);
		HAL_UART_Transmit(&huart3, buffFrame, strlen(buffFrame),HAL_MAX_DELAY);
	}

}







