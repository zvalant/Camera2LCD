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
#define OV5640_I2C_ADDR_W (0x78)
#define OV5640_I2C_ADDR_R (0x79)
#define OV5640_POLARITY 0x22

//Project consts

const uint16_t MAX_UART_DELAY = 1000;
extern DCMI_HandleTypeDef hdcmi;  // This should match what's in your main.c
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
	HAL_Delay(1);


	return HAL_I2C_Mem_Write(&hi2c1, OV5640_I2C_ADDR_W, regAddr,
			I2C_MEMADD_SIZE_16BIT,&data,1,HAL_MAX_DELAY);

}

HAL_StatusTypeDef OV5640_ReadReg(uint16_t regAddr, uint8_t *dataPtr) {
	char buff[25] = "readStart\r\n";
	HAL_UART_Transmit(&huart3, &buff, sizeof(buff), HAL_MAX_DELAY);
	return HAL_I2C_Mem_Read(&hi2c1, OV5640_I2C_ADDR_R, regAddr,
			I2C_MEMADD_SIZE_16BIT, dataPtr, 1, HAL_MAX_DELAY);
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
	const uint8_t heightBit_H = selectedHeight>>8;
	status |= OV5640_WriteReg(OV5640_REG_OUT_WIDTH_H, (uint8_t)selectedWidth>>8);
	status |= OV5640_WriteReg(OV5640_REG_OUT_WIDTH_L, (uint8_t)selectedWidth&0xFF);
	status |= OV5640_WriteReg(OV5640_REG_OUT_HEIGHT_H, heightBit_H);
	status |= OV5640_WriteReg(OV5640_REG_OUT_HEIGHT_L, (uint8_t)selectedHeight&0xFF);
	return status;
}
HAL_StatusTypeDef OV5640_SetFormat(void){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t formatData= OV5640_Formats[activeCameraConfigPtr->pixelFormat].formatValue;
	status |= OV5640_WriteReg(OV5640_REG_FORMAT_CTRL, formatData);
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

	status |= OV5640_WriteReg(OV5640_POLARITY_CTRL, OV5640_POLARITY);
	status |= OV5640_WriteReg(0x3017, 0xFF);
	uint8_t heightH = 0, heightL = 0;
	OV5640_ReadReg(OV5640_REG_OUT_HEIGHT_H, &heightH);
	OV5640_ReadReg(OV5640_REG_OUT_HEIGHT_L, &heightL);
	char buff3[30];
	sprintf(buff3, "Height H: %d, L: %d\n\r", heightH, heightL);
	HAL_UART_Transmit(&huart3, buff3, strlen(buff3), HAL_MAX_DELAY);
	uint8_t pol1;
	OV5640_ReadReg(OV5640_REG_FORMAT_CTRL, &pol1);
	char buff2[20];
	sprintf(buff2, "format: : %d\n\r", pol1);
	HAL_UART_Transmit(&huart3, buff2, strlen(buff2), HAL_MAX_DELAY);
	OV5640_WriteReg(0x503D, 0x80);
	HAL_Delay(10);
	uint16_t result = 0;
	uint8_t resultL = 0;
	uint8_t resultH = 0;
	//write format for format control register to make format rgb565
	char buff5[20];
	OV5640_ReadReg(OV5640_REG_OUT_HEIGHT_L, &resultL);
	OV5640_ReadReg(OV5640_REG_OUT_HEIGHT_H, &resultH);
	result = (resultH<<8)+resultL;

	sprintf(buff5, "result height: %d\n\r", result);
	HAL_UART_Transmit(&huart3, buff5, strlen(buff5), HAL_MAX_DELAY);
	uint8_t result1 = 0;
	OV5640_ReadReg(OV5640_REG_OUT_WIDTH_L, &result1);
	char buff4[20];
	sprintf(buff4, "width: %d\n\r", result1);

	HAL_UART_Transmit(&huart3, buff4, strlen(buff4), HAL_MAX_DELAY);
	return status;



}
void frameCapture(void){
	__HAL_RCC_DCMI_CLK_ENABLE();

	uint16_t width = OV5640_Resolutions[activeCameraConfigPtr->resolution].width;
	uint16_t height = OV5640_Resolutions[activeCameraConfigPtr->resolution].height;
	uint8_t pixelSize = OV5640_Formats[activeCameraConfigPtr->pixelFormat].bytesPerPixel;

	static uint32_t frameBuffer[240*320/2];

	// Enable DCMI
	DCMI->CR |= DCMI_CR_ENABLE;

	// Wait for VSYNC to go high (start of frame)
	while(!(DCMI->SR & DCMI_SR_VSYNC));

	// Wait for VSYNC to go low (active frame period)
	while(DCMI->SR & DCMI_SR_VSYNC);

	// Now capture data during active frame
	DCMI->CR |= DCMI_CR_CAPTURE;

	// Read some pixels when FIFO has data
	for(int i = 0; i < 10 && i < sizeof(frameBuffer)/4; i++) {
		// Wait for data to be available
		while(!(DCMI->SR & DCMI_SR_FNE));  // FIFO not empty
		frameBuffer[i] = DCMI->DR;

		char pixelBuff[30];
		sprintf(pixelBuff, "Pixel %d: 0x%08lX\n\r", i, frameBuffer[i]);
		HAL_UART_Transmit(&huart3, pixelBuff, strlen(pixelBuff), HAL_MAX_DELAY);
	}

}







