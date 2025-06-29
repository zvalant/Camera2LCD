/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"

#include "ov5640.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ILI9341_SendCommand(uint8_t cmd){
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,GPIO_PIN_SET);

}

void ILI9341_SendData(uint8_t *data, uint16_t length){
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, data, length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,GPIO_PIN_SET);
}

void LCDBlink_Test(void) {
    while(1) {
        // Turn display ON (should show whatever was last written)
        ILI9341_SendCommand(0x29);  // Display ON
        HAL_UART_Transmit(&huart3, "Display ON\r\n", 12, HAL_MAX_DELAY);
        HAL_Delay(1000);

        // Turn display OFF (should go black/blank)
        ILI9341_SendCommand(0x28);  // Display OFF
        HAL_UART_Transmit(&huart3, "Display OFF\r\n", 13, HAL_MAX_DELAY);
        HAL_Delay(1000);
    }
}
void ILI9341_Init(void) {
    // Hardware reset
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(150);

    // Basic initialization sequence
    ILI9341_SendCommand(0x01);  // Software reset
    HAL_Delay(10);
    char buff1[30];
    ILI9341_SendCommand(0xCF);  // Power Control B
    uint8_t init_data1[] = {0x00, 0xC1, 0x30};
    ILI9341_SendData(init_data1, 3);

    ILI9341_SendCommand(0x36);  // Memory Access Control
    uint8_t madctl = 0x48;      // RGB order, BGR encoding
    ILI9341_SendData(&madctl, 1);

    ILI9341_SendCommand(0x3A);  // Pixel Format
    uint8_t pixfmt = 0x55;      // 16-bit/pixel
    ILI9341_SendData(&pixfmt, 1);



    ILI9341_SendCommand(0x11);  // Sleep out
    HAL_Delay(120);

    ILI9341_SendCommand(0x29);  // Display on
    HAL_Delay(10);
}
void ILI9341_FillScreen(uint16_t color) {
    // Set column address (0 to 239)
    uint8_t col_cmd = 0x2A;
    uint8_t col_data[4] = {0x00, 0x00, 0x00, 0xEF};
    uint8_t page_cmd = 0x2B;
    uint8_t page_data[4] = {0x00, 0x00, 0x01, 0x3F};
    uint8_t mem_cmd = 0x2C;
    ILI9341_SendCommand(col_cmd);
    ILI9341_SendData(&col_data, 4);
    ILI9341_SendCommand(page_cmd);
    ILI9341_SendData(&page_data, 4);
    ILI9341_SendCommand(mem_cmd);
    uint8_t pixel_data[2] = {color >> 8, color & 0xFF};

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);     // Data mode
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);   // Select

    for(uint32_t i = 0; i < 240 * 320; i++) {
        HAL_SPI_Transmit(&hspi1, pixel_data, 2, HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);




// Deselect
}
void ILI9341_Init_Complete(void) {
    // Hardware reset
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(120);

    // Software reset
    ILI9341_SendCommand(0x01);  // SWRESET
    HAL_Delay(150);

    // Display off during initialization
    ILI9341_SendCommand(0x28);  // Display OFF

    // Power control sequence (from Adafruit library)
    ILI9341_SendCommand(0xCB);  // Power control B
    uint8_t pwctrlb[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
    ILI9341_SendData(pwctrlb, 5);

    ILI9341_SendCommand(0xCF);  // Power control A
    uint8_t pwctrla[] = {0x00, 0xC1, 0x30};
    ILI9341_SendData(pwctrla, 3);

    ILI9341_SendCommand(0xE8);  // Driver timing control A
    uint8_t dtca[] = {0x85, 0x00, 0x78};
    ILI9341_SendData(dtca, 3);

    ILI9341_SendCommand(0xEA);  // Driver timing control B
    uint8_t dtcb[] = {0x00, 0x00};
    ILI9341_SendData(dtcb, 2);

    ILI9341_SendCommand(0xED);  // Power on sequence control
    uint8_t pwseq[] = {0x64, 0x03, 0x12, 0x81};
    ILI9341_SendData(pwseq, 4);

    ILI9341_SendCommand(0xF7);  // Pump ratio control
    uint8_t pumprc[] = {0x20};
    ILI9341_SendData(pumprc, 1);

    // Main power control registers
    ILI9341_SendCommand(0xC0);  // Power control 1
    uint8_t pwctrl1[] = {0x23};  // VRH[5:0]
    ILI9341_SendData(pwctrl1, 1);

    ILI9341_SendCommand(0xC1);  // Power control 2
    uint8_t pwctrl2[] = {0x10};  // SAP[2:0];BT[3:0]
    ILI9341_SendData(pwctrl2, 1);

    ILI9341_SendCommand(0xC5);  // VCOM control 1
    uint8_t vmctrl1[] = {0x3E, 0x28};
    ILI9341_SendData(vmctrl1, 2);

    ILI9341_SendCommand(0xC7);  // VCOM control 2
    uint8_t vmctrl2[] = {0x86};
    ILI9341_SendData(vmctrl2, 1);

    // Memory and display settings
    ILI9341_SendCommand(0x36);  // Memory Access Control
    uint8_t madctl[] = {0x48};   // BGR color order, normal orientation
    ILI9341_SendData(madctl, 1);

    ILI9341_SendCommand(0x3A);  // Pixel format
    uint8_t pixfmt[] = {0x55};   // 16 bits per pixel (RGB565)
    ILI9341_SendData(pixfmt, 1);

    ILI9341_SendCommand(0xB1);  // Frame rate control
    uint8_t frmctrl[] = {0x00, 0x18};
    ILI9341_SendData(frmctrl, 2);

    ILI9341_SendCommand(0xB6);  // Display function control
    uint8_t dfunctr[] = {0x08, 0x82, 0x27};
    ILI9341_SendData(dfunctr, 3);

    ILI9341_SendCommand(0xF2);  // 3Gamma function disable
    uint8_t gamma3g[] = {0x00};
    ILI9341_SendData(gamma3g, 1);

    ILI9341_SendCommand(0x26);  // Gamma curve selected
    uint8_t gamset[] = {0x01};
    ILI9341_SendData(gamset, 1);

    // Positive gamma correction
    ILI9341_SendCommand(0xE0);
    uint8_t pgamma[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                        0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
    ILI9341_SendData(pgamma, 15);

    // Negative gamma correction
    ILI9341_SendCommand(0xE1);
    uint8_t ngamma[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                        0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
    ILI9341_SendData(ngamma, 15);

    // Exit sleep mode
    ILI9341_SendCommand(0x11);  // SLPOUT
    HAL_Delay(120);

    // Display on
    ILI9341_SendCommand(0x29);  // DISPON
    HAL_Delay(100);

    HAL_UART_Transmit(&huart3, "ILI9341 Init Complete!\r\n", 24, HAL_MAX_DELAY);
}

// Basic test function to verify initialization
void ILI9341_TestDisplay(void) {
    // Fill screen with different colors to test
    ILI9341_FillScreen(0xF800);  // Red
    HAL_Delay(1000);
    ILI9341_FillScreen(0x07E0);  // Green
    HAL_Delay(1000);
    ILI9341_FillScreen(0x001F);  // Blue
    HAL_Delay(1000);
    ILI9341_FillScreen(0xFFFF);  // White
    HAL_Delay(1000);
    ILI9341_FillScreen(0x0000);  // Black
}
void TestSPIBasic(void) {
    HAL_UART_Transmit(&huart3, "Testing basic SPI...\r\n", 22, HAL_MAX_DELAY);

    uint8_t testByte = 0xAA;
    HAL_StatusTypeDef status;

    // Test SPI transmission
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);  // Select
    status = HAL_SPI_Transmit(&hspi1, &testByte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);    // Deselect

    char spiMsg[50];
    sprintf(spiMsg, "SPI Status: %d (0=OK)\r\n", status);
    HAL_UART_Transmit(&huart3, spiMsg, strlen(spiMsg), HAL_MAX_DELAY);
}
void TestGPIOPins(void) {
    HAL_UART_Transmit(&huart3, "Testing GPIO pins...\r\n", 22, HAL_MAX_DELAY);

    // Test CS pin - should control chip select
    for(int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);  // LOW
        HAL_Delay(500);
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);    // HIGH
        HAL_Delay(500);
        char msg[30];
        sprintf(msg, "CS Toggle %d\r\n", i);
        HAL_UART_Transmit(&huart3, msg, strlen(msg), HAL_MAX_DELAY);
    }

    // Test RESET pin - this should cause visible display reset
    HAL_UART_Transmit(&huart3, "Testing RESET pin...\r\n", 22, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);  // Should reset display
    HAL_Delay(2000);  // Display should go off/change
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);    // Should restore display
    HAL_Delay(2000);

    HAL_UART_Transmit(&huart3, "GPIO test complete\r\n", 21, HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  extern DCMI_HandleTypeDef hdcmi;
  HAL_StatusTypeDef cameraConfigStatus = OV5640_PowerUpSequence();
  uint8_t reg1 = 0;
  OV5640_ReadReg(OV5640_POLARITY_CTRL,&reg1);
  char buffReg1[20];
  sprintf(buffReg1, "Polarity: %d\r\n", reg1);
  HAL_UART_Transmit(&huart3, buffReg1, strlen(buffReg1), HAL_MAX_DELAY);
	if (cameraConfigStatus == HAL_OK){
		HAL_GPIO_WritePin(I2C_SUCCESS_GPIO_Port, I2C_SUCCESS_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(I2C_SUCCESS_GPIO_Port, I2C_SUCCESS_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);
	}

	frameCapture();
	char buffFrame[20];
	sprintf(buffFrame, "Start of image: 0x%08lX,\n\r", DCMI->DR);
	HAL_UART_Transmit(&huart3, buffFrame, strlen(buffFrame),HAL_MAX_DELAY);
	ILI9341_Init_Complete();
	ILI9341_TestDisplay();
	TestSPIBasic();
	TestGPIOPins();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 4;
  RCC_OscInitStruct.PLL.PLLQ = 24;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVQ);
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
