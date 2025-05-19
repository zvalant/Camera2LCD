/*
 * OV5640.h
 *
 *  Created on: May 11, 2025
 *      Author: zac23
 */

#ifndef INC_OV5640_H_
#define INC_OV5640_H_


// register addresses

#define OV5640_REG_CHIP_ID_H 0x300A
#define OV5640_REG_CHIP_ID_L 0x300B
#define OV5640_SOFTWARE_RESET 0X3008
#define OV5640_REG_FORMAT_CTRL 0x4300
#define OV5640_REG_OUT_WIDTH_H 0x3808
#define OV5640_REG_OUT_WIDTH_L 0x3809
#define OV5640_REG_OUT_HEIGHT_H 0x380A
#define OV5640_REG_OUT_HEIGHT_L 0x380B

typedef enum {
	OV5640_RES_240X320,
	OV5640_RES_480X640,
	OV5640_RES_720X1080
} OV5640_Resolution;

typedef enum {
	OV5640_FORMAT_RGB565,
	OV5640_FORMAT_YUV422,
	OV5640_FORMAT_JPEG,
	OV5640_FORMAT_RAW
}OV5640_Format;

typedef struct {
	const OV5640_Resolution resolution;
	const OV5640_Format pixelFormat;
} OV5640_CameraConfig;

extern OV5640_CameraConfig activeCameraConfig;
extern OV5640_CameraConfig *activeCameraConfigPtr;


#include "stm32h7xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef OV5640_WriteReg(uint16_t regAddr, uint8_t data);
HAL_StatusTypeDef OV5640_ReadReg(uint16_t regAddr, uint8_t *dataPtr);
HAL_StatusTypeDef OV5640_TestConnection(void);
HAL_StatusTypeDef OV5640_SetResolution(void);
HAL_StatusTypeDef OV5640_SetPixelFormat(void);
HAL_StatusTypeDef OV5640_ConfigureCamera(void);
HAL_StatusTypeDef OV5640_PowerUpSequence(void);
void frameCapture(void);



#endif /* INC_OV5640_H_ */
