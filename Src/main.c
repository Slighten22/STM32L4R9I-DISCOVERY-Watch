/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gfxmmu_lut.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
DCMI_HandleTypeDef hdcmi;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

GFXMMU_HandleTypeDef hgfxmmu;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

OSPI_HandleTypeDef hospi1;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;

SD_HandleTypeDef hsd1;

SRAM_HandleTypeDef hsram4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_RTC_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_DCMI_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_GFXMMU_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */
#define GFXMMU_LCD_SIZE 390
uint8_t image[4*390*390];
static void myMX_LTDC_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LTDC_Init();
  MX_RTC_Init();
  MX_DSIHOST_DSI_Init();
  MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_OCTOSPI1_Init();
  MX_DCMI_Init();
  MX_DFSDM1_Init();
  MX_FMC_Init();
  MX_DMA2D_Init();
  MX_GFXMMU_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_OSPI
                              |RCC_PERIPHCLK_DSI|RCC_PERIPHCLK_LTDC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.DsiClockSelection = RCC_DSICLKSOURCE_DSIPHY;
  PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  PeriphClkInit.LtdcClockSelection = RCC_LTDCCLKSOURCE_PLLSAI2_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 2;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 8;
  PeriphClkInit.PLLSAI2.PLLSAI2P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_LTDCCLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_EMBEDDED;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.SynchroCode.FrameEndCode = 0;
  hdcmi.Init.SynchroCode.FrameStartCode = 0;
  hdcmi.Init.SynchroCode.LineStartCode = 0;
  hdcmi.Init.SynchroCode.LineEndCode = 0;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 2;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0x00;
  hdfsdm1_channel0.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.Init.BytesSwap = DMA2D_BYTES_REGULAR;
  hdma2d.Init.LineOffsetMode = DMA2D_LOM_PIXELS;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 32;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 16;
  PhyTimings.ClockLaneLP2HSTime = 10;
  PhyTimings.DataLaneHS2LPTime = 8;
  PhyTimings.DataLaneLP2HSTime = 6;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 390;
  CmdCfg.TearingEffectSource = DSI_TE_DSILINK;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief GFXMMU Initialization Function
  * @param None
  * @retval None
  */
static void MX_GFXMMU_Init(void)
{

  /* USER CODE BEGIN GFXMMU_Init 0 */

  /* USER CODE END GFXMMU_Init 0 */

  /* USER CODE BEGIN GFXMMU_Init 1 */

  /* USER CODE END GFXMMU_Init 1 */
  hgfxmmu.Instance = GFXMMU;
  hgfxmmu.Init.BlocksPerLine = GFXMMU_192BLOCKS;
  hgfxmmu.Init.DefaultValue = 0;
  hgfxmmu.Init.Buffers.Buf0Address = 0;
  hgfxmmu.Init.Buffers.Buf1Address = 0;
  hgfxmmu.Init.Buffers.Buf2Address = 0;
  hgfxmmu.Init.Buffers.Buf3Address = 0;
  hgfxmmu.Init.Interrupts.Activation = ENABLE;
  if (HAL_GFXMMU_Init(&hgfxmmu) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_GFXMMU_ConfigLut(&hgfxmmu, GFXMMU_LUT_FIRST, GFXMMU_LUT_SIZE, (uint32_t)gfxmmu_lut_config) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GFXMMU_Init 2 */

  /* USER CODE END GFXMMU_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 404;
  hltdc.Init.AccumulatedActiveH = 395;
  hltdc.Init.TotalWidth = 410;
  hltdc.Init.TotalHeigh = 397;
  hltdc.Init.Backcolor.Blue = 128;
  hltdc.Init.Backcolor.Green = 128;
  hltdc.Init.Backcolor.Red = 128;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = GFXMMU_VIRTUAL_BUFFER0_BASE;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = GFXMMU_VIRTUAL_BUFFER0_BASE;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_LTDC_SetPitch(&hltdc, 768, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_LTDC_SetPitch(&hltdc, 768, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 1;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 256;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 1;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
  sMemMappedCfg.TimeOutPeriod = 1;
  if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength = 8;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM4 memory initialization sequence
  */
  hsram4.Instance = FMC_NORSRAM_DEVICE;
  hsram4.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram4.Init */
  hsram4.Init.NSBank = FMC_NORSRAM_BANK2;
  hsram4.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram4.Init.MemoryType = FMC_MEMORY_TYPE_PSRAM;
  hsram4.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram4.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram4.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram4.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram4.Init.WriteOperation = FMC_WRITE_OPERATION_DISABLE;
  hsram4.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram4.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram4.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram4.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram4.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram4.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram4.Init.NBLSetupTime = 0;
  hsram4.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.DataHoldTime = 0;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram4, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
DSI_HandleTypeDef                    hdsi;

typedef enum
{
   IO_MODE_INPUT = 0,   /* input floating */
   IO_MODE_OUTPUT,      /* output Push Pull */
   IO_MODE_IT_RISING_EDGE,   /* float input - irq detect on rising edge */
   IO_MODE_IT_FALLING_EDGE,  /* float input - irq detect on falling edge */
   IO_MODE_IT_LOW_LEVEL,     /* float input - irq detect on low level */
   IO_MODE_IT_HIGH_LEVEL,    /* float input - irq detect on high level */
   /* following modes only available on MFX*/
   IO_MODE_ANALOG,           /* analog mode */
   IO_MODE_OFF,              /* when pin isn't used*/
   IO_MODE_INPUT_PU,         /* input with internal pull up resistor */
   IO_MODE_INPUT_PD,         /* input with internal pull down resistor */
   IO_MODE_OUTPUT_OD,          /* Open Drain output without internal resistor */
   IO_MODE_OUTPUT_OD_PU,       /* Open Drain output with  internal pullup resistor */
   IO_MODE_OUTPUT_OD_PD,       /* Open Drain output with  internal pulldown resistor */
   IO_MODE_OUTPUT_PP,          /* PushPull output without internal resistor */
   IO_MODE_OUTPUT_PP_PU,       /* PushPull output with  internal pullup resistor */
   IO_MODE_OUTPUT_PP_PD,       /* PushPull output with  internal pulldown resistor */
   IO_MODE_IT_RISING_EDGE_PU,   /* push up resistor input - irq on rising edge  */
   IO_MODE_IT_RISING_EDGE_PD,   /* push dw resistor input - irq on rising edge  */
   IO_MODE_IT_FALLING_EDGE_PU,  /* push up resistor input - irq on falling edge */
   IO_MODE_IT_FALLING_EDGE_PD,  /* push dw resistor input - irq on falling edge */
   IO_MODE_IT_LOW_LEVEL_PU,     /* push up resistor input - irq detect on low level */
   IO_MODE_IT_LOW_LEVEL_PD,     /* push dw resistor input - irq detect on low level */
   IO_MODE_IT_HIGH_LEVEL_PU,    /* push up resistor input - irq detect on high level */
   IO_MODE_IT_HIGH_LEVEL_PD,    /* push dw resistor input - irq detect on high level */

}IO_ModeTypedef;

#define IO_PIN_0                  ((uint32_t)0x00000001)
#define IO_PIN_1                  ((uint32_t)0x00000002)
#define IO_PIN_2                  ((uint32_t)0x00000004)
#define IO_PIN_3                  ((uint32_t)0x00000008)
#define IO_PIN_4                  ((uint32_t)0x00000010)
#define IO_PIN_5                  ((uint32_t)0x00000020)
#define IO_PIN_6                  ((uint32_t)0x00000040)
#define IO_PIN_7                  ((uint32_t)0x00000080)
#define IO_PIN_8                  ((uint32_t)0x00000100)
#define IO_PIN_9                  ((uint32_t)0x00000200)
#define IO_PIN_10                 ((uint32_t)0x00000400)
#define IO_PIN_11                 ((uint32_t)0x00000800)
#define IO_PIN_12                 ((uint32_t)0x00001000)
#define IO_PIN_13                 ((uint32_t)0x00002000)
#define IO_PIN_14                 ((uint32_t)0x00004000)
#define IO_PIN_15                 ((uint32_t)0x00008000)
#define AGPIO_PIN_0               ((uint32_t)0x00010000)
#define AGPIO_PIN_1               ((uint32_t)0x00020000)
#define AGPIO_PIN_2               ((uint32_t)0x00040000)
#define IO_PIN_ALL                ((uint32_t)0x0003FFFF)

static void LCD_PowerOff(void)
{
    BSP_IO_Init();

    /* Activate DSI_RESET */
    BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

    /* Wait at least 5 ms */
    HAL_Delay(5);

    /* Set DSI_POWER_ON to analog mode only if psram is not currently used */
#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Disable first DSI_1V8_PWRON then DSI_3V3_PWRON */
    BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_ANALOG);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
}

volatile int k = 0;

/**
  * @brief  LCD power on
  *         Power on LCD.
  */
static void LCD_PowerOn(void)
{
    BSP_IO_Init();

#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    /* Set DSI_POWER_ON to input floating to avoid I2C issue during input PD configuration */
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_INPUT);

    /* Configure the GPIO connected to DSI_RESET signal */
    BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);

    /* Activate DSI_RESET (active low) */
    BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

    /* Configure the GPIO connected to DSI_POWER_ON signal as input pull down */
    /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_INPUT_PD);

    /* Wait at least 1ms before enabling 1V8_LCD */
    HAL_Delay(1);

    /* Configure the GPIO connected to DSI_POWER_ON signal as output low */
    /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Configure the GPIO connected to DSI_3V3_POWERON signal as output low */
    /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_SET);
    //HAL_Delay(1000);
    BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);
    //HAL_Delay(1000);
    BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);

    /* Wait at least 1ms before enabling 1V8_LCD */
    HAL_Delay(1);

    /* Configure the GPIO connected to DSI_1V8_POWERON signal as output low */
    /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
    BSP_IO_WritePin(AGPIO_PIN_2, GPIO_PIN_SET);
    BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_OUTPUT);
    BSP_IO_WritePin(AGPIO_PIN_2, GPIO_PIN_RESET);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */

    /* Wait at least 15 ms (minimum reset low width is 10ms and add margin for 1V8_LCD ramp-up) */
    HAL_Delay(15);

#if defined(USE_STM32L4R9I_DISCO_REVA) || defined(USE_STM32L4R9I_DISCO_REVB)
    /* Desactivate DSI_RESET */
    BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_SET);
#else /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */
    /* Configure the GPIO connected to DSI_RESET signal */
    BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);

    /* Desactivate DSI_RESET */
    BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_SET);
#endif /* USE_STM32L4R9I_DISCO_REVA || USE_STM32L4R9I_DISCO_REVB */

    /* Wait reset complete time (maximum time is 5ms when LCD in sleep mode and 120ms when LCD is not in sleep mode) */
    HAL_Delay(120);
}



static void myMX_LTDC_Init(void)
{

    uint8_t ScanLineParams[2];
    uint16_t scanline = (GFXMMU_LCD_SIZE - 10);

    DSI_PLLInitTypeDef      dsiPllInit;
    DSI_PHY_TimerTypeDef    PhyTimings;
    DSI_HOST_TimeoutTypeDef HostTimeouts;
    DSI_LPCmdTypeDef        LPCmd;
    DSI_CmdCfgTypeDef       CmdCfg;
    LTDC_LayerCfgTypeDef    LayerCfg;

    /* Power off then on on LCD */
    LCD_PowerOff();
    LCD_PowerOn();




  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  /* LTDC initialization */
     hltdc.Instance = LTDC;
     __HAL_LTDC_RESET_HANDLE_STATE(&hltdc);
     hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
     hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
     hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
     hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
     hltdc.Init.HorizontalSync = 0;   /* HSYNC width - 1 */
     hltdc.Init.VerticalSync = 0;   /* VSYNC width - 1 */
     hltdc.Init.AccumulatedHBP = 1;   /* HSYNC width + HBP - 1 */
     hltdc.Init.AccumulatedVBP = 1;   /* VSYNC width + VBP - 1 */
     hltdc.Init.AccumulatedActiveW = GFXMMU_LCD_SIZE + 1; /* HSYNC width + HBP + Active width - 1 */
     hltdc.Init.AccumulatedActiveH = GFXMMU_LCD_SIZE + 1; /* VSYNC width + VBP + Active height - 1 */
     hltdc.Init.TotalWidth = GFXMMU_LCD_SIZE + 2; /* HSYNC width + HBP + Active width + HFP - 1 */
     hltdc.Init.TotalHeigh = GFXMMU_LCD_SIZE + 2; /* VSYNC width + VBP + Active height + VFP - 1 */
     hltdc.Init.Backcolor.Red = 0; /* NU default value */
     hltdc.Init.Backcolor.Green = 0; /* NU default value */
     hltdc.Init.Backcolor.Blue = 0; /* NU default value */
     hltdc.Init.Backcolor.Reserved = 0xFF;
     if (HAL_LTDC_Init(&hltdc) != HAL_OK)
     {
         Error_Handler();
     }

     /* LTDC layers configuration */
         pLayerCfg.WindowX0 = 0;
         pLayerCfg.WindowX1 = 0;
         pLayerCfg.WindowY0 = 0;
         pLayerCfg.WindowY1 = 0;
         pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
         pLayerCfg.Alpha = 0xFF; /* NU default value */
         pLayerCfg.Alpha0 = 0; /* NU default value */
         pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA; /* NU default value */
         pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA; /* NU default value */
         pLayerCfg.FBStartAdress = (uint32_t)image;
         pLayerCfg.ImageWidth = 390; /* virtual frame buffer contains 768 pixels per line for 32bpp */
         pLayerCfg.ImageHeight = 390;
         pLayerCfg.Backcolor.Red = 0; /* NU default value */
         pLayerCfg.Backcolor.Green = 0; /* NU default value */
         pLayerCfg.Backcolor.Blue = 0; /* NU default value */
         pLayerCfg.Backcolor.Reserved = 0xFF;
         if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
         {
             Error_Handler();
         }
















         /*********************/
         /* DSI CONFIGURATION */
         /*********************/

         /* DSI initialization */
         hdsi.Instance = DSI;
         /* PATCH PNA */
         //  __HAL_DSI_RESET_HANDLE_STATE(&hdsi);
         hdsi.State = HAL_DSI_STATE_RESET;
         /* END PATCH PNA */
         hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
         /* We have 1 data lane at 500Mbps => lane byte clock at 500/8 = 62,5 MHZ */
         /* We want TX escape clock at arround 20MHz and under 20MHz so clock division is set to 4 */
         hdsi.Init.TXEscapeCkdiv = 4;
         hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
         /* We have HSE value at 16 Mhz and we want data lane at 500Mbps */
         dsiPllInit.PLLNDIV = 125;
         dsiPllInit.PLLIDF = DSI_PLL_IN_DIV4;
         dsiPllInit.PLLODF = DSI_PLL_OUT_DIV1;
         if (HAL_DSI_Init(&hdsi, &dsiPllInit) != HAL_OK)
         {
             Error_Handler();
         }

         PhyTimings.ClockLaneHS2LPTime = 33; /* Tclk-post + Tclk-trail + Ths-exit = [(60ns + 52xUI) + (60ns) + (300ns)]/16ns */
         PhyTimings.ClockLaneLP2HSTime = 30; /* Tlpx + (Tclk-prepare + Tclk-zero) + Tclk-pre = [150ns + 300ns + 8xUI]/16ns */
         PhyTimings.DataLaneHS2LPTime = 11; /* Ths-trail + Ths-exit = [(60ns + 4xUI) + 100ns]/16ns */
         PhyTimings.DataLaneLP2HSTime = 21; /* Tlpx + (Ths-prepare + Ths-zero) + Ths-sync = [150ns + (145ns + 10xUI) + 8xUI]/16ns */
         PhyTimings.DataLaneMaxReadTime = 0;
         PhyTimings.StopWaitTime = 7;
         if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
         {
             Error_Handler();
         }

         HostTimeouts.TimeoutCkdiv = 1;
         HostTimeouts.HighSpeedTransmissionTimeout = 0;
         HostTimeouts.LowPowerReceptionTimeout = 0;
         HostTimeouts.HighSpeedReadTimeout = 0;
         HostTimeouts.LowPowerReadTimeout = 0;
         HostTimeouts.HighSpeedWriteTimeout = 0;
         HostTimeouts.HighSpeedWritePrespMode = 0;
         HostTimeouts.LowPowerWriteTimeout = 0;
         HostTimeouts.BTATimeout = 0;
         if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
         {
             Error_Handler();
         }

         LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_ENABLE;
         LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
         LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
         LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_ENABLE;
         LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_ENABLE;
         LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_ENABLE;
         LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
         LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_ENABLE;
         LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
         LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_ENABLE;
         LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
         LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
         LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
         if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
         {
             Error_Handler();
         }

         CmdCfg.VirtualChannelID = 0;
         CmdCfg.ColorCoding = DSI_RGB888;
         CmdCfg.CommandSize = GFXMMU_LCD_SIZE;
         CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
         CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
         CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
         CmdCfg.AutomaticRefresh = DSI_AR_DISABLE;
         CmdCfg.TearingEffectPolarity = DSI_TE_FALLING_EDGE;
         CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
         CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
         CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_ENABLE;
         if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
         {
             Error_Handler();
         }

         //  /* Disable the Tearing Effect interrupt activated by default on previous function */
         //  __HAL_DSI_DISABLE_IT(&hdsi, DSI_IT_TE);

         if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
         {
             Error_Handler();
         }

         /* Enable DSI */
         __HAL_DSI_ENABLE(&hdsi);

         /*************************/
         /* LCD POWER ON SEQUENCE */
         /*************************/
         /* Step 1 */
         /* Go to command 2 */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x01);
         /* IC Frame rate control, set power, sw mapping, mux swithc timing command */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x06, 0x62);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0E, 0x80);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0F, 0x80);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x10, 0x71);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x13, 0x81);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x14, 0x81);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x15, 0x82);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x16, 0x82);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x18, 0x88);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x19, 0x55);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1A, 0x10);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1C, 0x99);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1D, 0x03);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1E, 0x03);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1F, 0x03);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x20, 0x03);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x25, 0x03);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x26, 0x8D);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2A, 0x03);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2B, 0x8D);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x37, 0x10);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3A, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3B, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3D, 0x20);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3F, 0x3A);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x40, 0x30);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x41, 0x1A);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x42, 0x33);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x43, 0x22);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x44, 0x11);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x45, 0x66);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x46, 0x55);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x47, 0x44);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4C, 0x33);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4D, 0x22);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4E, 0x11);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x4F, 0x66);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x50, 0x55);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x51, 0x44);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x57, 0x33);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x6B, 0x1B);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x70, 0x55);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x74, 0x0C);

         /* Go to command 3 */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x02);
         /* Set the VGMP/VGSP coltage control */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9B, 0x40);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9C, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9D, 0x20);

         /* Go to command 4 */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x03);
         /* Set the VGMP/VGSP coltage control */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9B, 0x40);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9C, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x9D, 0x20);

         /* Go to command 5 */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x04);
         /* VSR command */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x5D, 0x10);
         /* VSR1 timing set */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x00, 0x8D);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x01, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x02, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x03, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x04, 0x10);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x05, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x06, 0xA7);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x07, 0x20);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x08, 0x00);
         /* VSR2 timing set */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x09, 0xC2);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0A, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0B, 0x02);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0C, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0D, 0x40);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0E, 0x06);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x0F, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x10, 0xA7);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x11, 0x00);
         /* VSR3 timing set */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x12, 0xC2);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x13, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x14, 0x02);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x15, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x16, 0x40);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x17, 0x07);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x18, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x19, 0xA7);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1A, 0x00);
         /* VSR4 timing set */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1B, 0x82);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1C, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1D, 0xFF);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1E, 0x05);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x1F, 0x60);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x20, 0x02);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x21, 0x01);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x22, 0x7C);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x23, 0x00);
         /* VSR5 timing set */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x24, 0xC2);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x25, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x26, 0x04);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x27, 0x02);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x28, 0x70);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x29, 0x05);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2A, 0x74);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2B, 0x8D);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2D, 0x00);
         /* VSR6 timing set */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2F, 0xC2);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x30, 0x00);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x31, 0x04);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x32, 0x02);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x33, 0x70);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x34, 0x07);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x74);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x8D);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x37, 0x00);
         /* VSR marping command */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x5E, 0x20);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x5F, 0x31);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x60, 0x54);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x61, 0x76);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x62, 0x98); /* �0x62� is suggested instead of �0x52� */

         /* Go to command 6 */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x05);
         /* Set the ELVSS voltage */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x05, 0x17); /* �0x17� is suggested instead of �0x07� for SM3321 */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x2A, 0x04);
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x91, 0x00);

         /* Go back in standard commands */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xFE, 0x00);

         /* Set tear off */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, DSI_SET_TEAR_OFF, 0x0);

         /* Set DSI mode to internal timing added vs ORIGINAL for Command mode */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xC2, 0x0);

         /* Set memory address MODIFIED vs ORIGINAL */
         uint8_t InitParam1[4] = { 0x00, 0x04, 0x01, 0x89 }; // MODIF OFe: adjusted w/ real image
         HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, DSI_SET_COLUMN_ADDRESS, InitParam1);
         uint8_t InitParam2[4] = { 0x00, 0x00, 0x01, 0x85 };
         HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, DSI_SET_PAGE_ADDRESS, InitParam2);

         /* Sleep out */
         HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, DSI_EXIT_SLEEP_MODE, 0x0);

         HAL_Delay(120);

         /* Set display on */
         if (HAL_DSI_ShortWrite(&hdsi,
                                0,
                                DSI_DCS_SHORT_PKT_WRITE_P0,
                                DSI_SET_DISPLAY_ON,
                                0x0) != HAL_OK)
         {
             Error_Handler();
         }

         /* Enable DSI Wrapper */
         __HAL_DSI_WRAPPER_ENABLE(&hdsi);

         ScanLineParams[0] = scanline >> 8;
         ScanLineParams[1] = scanline & 0x00FF;

         HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 2, DSI_SET_TEAR_SCANLINE, ScanLineParams);

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
