/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DELAY_BETWEEN_DXL_PACKET 0.003 //0.01:20Hz
#include "../../ScueDK/scuedk_embedded/master/MasterSerialLine.h"
#include "../../ScueDK/structs/structs.h"
#include "../../Sequence/inc/sequence.h"

using namespace scue;
using namespace seq;
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

int loopCnt;
int sequenceCnt=0;
double currentTime;
uint8_t uart1RxBuffer;
uint8_t uart2RxBuffer;
MasterSerialLine serial;
bool manipulatorRelay = true, baseRelay = true;
Master m;
vector<uint8_t> packet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	initMaster(&m);

	Sequence initSequence
	(
		"initDxl",
		make_shared<block::Function>([&]
		{
			serial.setActive(false);
			initManipulator(&m.manipulator);
		}),
		make_shared<block::Function>([&]
		{
			//set MX64 velocity profile to non-zero value to enable acceleration profile
			packet.clear();
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);
			packet.push_back(0xFE);
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0x93);
			//id13
			packet.push_back(13);
			packet.push_back(112);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, 32767, 4);
			//id14
			packet.push_back(14);
			packet.push_back(112);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, 32767, 4);
			//id15
			packet.push_back(15);
			packet.push_back(112);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, 32767, 4);

			Util::setLength(packet);
			Util::attachCRC(packet);
			HAL_UART_Transmit(&huart2, &packet[0], packet.size(),1);
		}),
		make_shared<block::Delay>(DELAY_BETWEEN_DXL_PACKET),
		make_shared<block::Function>([&]
		{
			//enable mx64s' torque
			packet.clear();
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);
			packet.push_back(0xFE);
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0X93);
			//id10
			packet.push_back(10);
			packet.push_back(0);
			packet.push_back(2);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);
			//id11
			packet.push_back(11);
			packet.push_back(0);
			packet.push_back(2);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);
			//id12
			packet.push_back(12);
			packet.push_back(0);
			packet.push_back(2);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);
			//id13
			packet.push_back(13);
			packet.push_back(64);
			packet.push_back(0);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);
			//id14
			packet.push_back(14);
			packet.push_back(64);
			packet.push_back(0);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);
			//id15
			packet.push_back(15);
			packet.push_back(64);
			packet.push_back(0);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);
			//id16
			packet.push_back(16);
			packet.push_back(64);
			packet.push_back(0);
			packet.push_back(1);
			packet.push_back(0);
			packet.push_back(1);

			Util::setLength(packet);
			Util::attachCRC(packet);
			HAL_UART_Transmit(&huart2, &packet[0], packet.size(),1);
		}),
		make_shared<block::Delay>(DELAY_BETWEEN_DXL_PACKET),
		make_shared<block::StartSequence>("dxl")
	);

	initSequence.compile();
	initSequence.start();

	Sequence dxlSequence
	(
		"dxl",
		make_shared<block::Function>([&]
		{
			serial.setActive(false);
		}),
		make_shared<block::Function>([&]
		{
			//set pro acceleration
			packet.clear();
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);
			packet.push_back(0xFE);
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0x93);
			//id10
			packet.push_back(10);
			packet.push_back(0x2C);
			packet.push_back(0x02);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, m.manipulator.targetAcceleration[0]*9.549296596425384, 4);
			//id11
			packet.push_back(11);
			packet.push_back(0x2C);
			packet.push_back(0x02);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, m.manipulator.targetAcceleration[1]*9.549296596425384, 4);
			//id12
			packet.push_back(12);
			packet.push_back(0x2C);
			packet.push_back(0x02);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, m.manipulator.targetAcceleration[2]*9.549296596425384, 4);

			Util::setLength(packet);
			Util::attachCRC(packet);
			HAL_UART_Transmit(&huart2, &packet[0], packet.size(),1);
		}),
		make_shared<block::Delay>(DELAY_BETWEEN_DXL_PACKET),
		make_shared<block::Function>([&]
		{
			//set MX64 acceleration
			packet.clear();
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);
			packet.push_back(0xFE);
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0x93);
			//id13
			packet.push_back(13);
			packet.push_back(0X6C);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, m.manipulator.targetAcceleration[3], 4);
			//id14
			packet.push_back(14);
			packet.push_back(0x6C);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, m.manipulator.targetAcceleration[4], 4);
			//id15
			packet.push_back(15);
			packet.push_back(0X6C);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, m.manipulator.targetAcceleration[5], 4);

			Util::setLength(packet);
			Util::attachCRC(packet);
			HAL_UART_Transmit(&huart2, &packet[0], packet.size(),1);
		}),
		make_shared<block::Delay>(DELAY_BETWEEN_DXL_PACKET),
		make_shared<block::Function>([&]
		{
			//bulk write to mx64s
			packet.clear();
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);
			packet.push_back(0xFE);
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0x93);
			//id13
			packet.push_back(13);
			packet.push_back(116);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.targetPosition[3], -90, 90, 1024, 3072), 4);
			//id14
			packet.push_back(14);
			packet.push_back(116);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.targetPosition[4], -90, 90, 1024, 3072), 4);
			//id15
			packet.push_back(15);
			packet.push_back(116);
			packet.push_back(0);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.targetPosition[5], -90, 90, 1024, 3072), 4);
			//id16 : gripper
			packet.push_back(16);
			packet.push_back(102);
			packet.push_back(0);
			packet.push_back(2);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.gripperTargetCurrent, -4, 4, -1193 , 1193), 4);

			Util::setLength(packet);
			Util::attachCRC(packet);
			HAL_UART_Transmit(&huart2, &packet[0], packet.size(),1);
		}),
		make_shared<block::Delay>(DELAY_BETWEEN_DXL_PACKET),
		make_shared<block::Function>([&]
		{
			//bulk write to pros
			packet.clear();
			packet.push_back(0xFF);
			packet.push_back(0xFF);
			packet.push_back(0xFD);
			packet.push_back(0x00);
			packet.push_back(0xFE);
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0);	//length placeholder. valid value is set by setLength()
			packet.push_back(0x93);
			//id10
			packet.push_back(10);
			packet.push_back(0x34);
			packet.push_back(0x02);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.targetPosition[0], -180, 180, -251417, 251417), 4);
			//id11
			packet.push_back(11);
			packet.push_back(0x34);
			packet.push_back(0x02);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.targetPosition[1], -180, 180, -501923, 501923), 4);
			//id12
			packet.push_back(12);
			packet.push_back(0x34);
			packet.push_back(0x02);
			packet.push_back(4);
			packet.push_back(0);
			Util::attachByteArray(packet, (int)Util::map(m.manipulator.targetPosition[2], -180, 180, -501923, 501923), 4);

			Util::setLength(packet);
			Util::attachCRC(packet);
			HAL_UART_Transmit(&huart2, &packet[0], packet.size(),1);
		}),
		make_shared<block::Delay>(DELAY_BETWEEN_DXL_PACKET),
		make_shared<block::Function>([&]
		{
			serial.setActive(true);
			serial.jumpToUplink();
			sequenceCnt++;
		})
	);

	dxlSequence.compile();

	serial.setExternalCallback([&]
	{
		Sequence* dxmseq = Sequence::getSequenceByName("dxl");
		if(dxmseq!=nullptr && !dxmseq->isRunning())dxmseq->start();
	});

	serial.setCycleCallback([&]
	{
		if(m.masterTweak.initManipulatorTrigger)
		{
			m.masterTweak.initManipulatorTrigger = false;
			Sequence* initDxlSeq = Sequence::getSequenceByName("initDxl");
			if(initDxlSeq!=nullptr && !initDxlSeq->isRunning()) initDxlSeq->start();
		}
	});
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
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	serial.init((uint8_t*)&m,sizeof(Master), 1, &huart1, &huart2);
	serial.addSlave(MasterSerialLine::Slave(1, (uint8_t*)&m.flipperController - (uint8_t*)&m, sizeof(m.flipperController), false));
	serial.addSlave(MasterSerialLine::Slave(2, (uint8_t*)&m.trackController - (uint8_t*)&m, sizeof(m.trackController), false));
	serial.addSlave(MasterSerialLine::Slave(3, (uint8_t*)&m.manipulator - (uint8_t*)&m, sizeof(m.manipulator), true));
	serial.addSlave(MasterSerialLine::Slave(4, (uint8_t*)&m.masterTweak - (uint8_t*)&m, sizeof(m.masterTweak), true));

	HAL_UART_Receive_DMA(&huart1, &uart1RxBuffer, 1);
	HAL_UART_Receive_DMA(&huart2, &uart2RxBuffer, 1);
	HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_UART_Receive_DMA(&huart3, &rxBuffer, 1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	loopCnt++;
	serial.update();
	Sequence::spinOnce(0.001);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)//PC SIDE
	{
		serial.pushPcRx(uart1RxBuffer);
		HAL_UART_Receive_DMA(&huart1, &uart1RxBuffer, 1);
	}
	if(huart->Instance == huart2.Instance)//SLAVE SIDE
	{
		serial.pushSlaveRx(uart2RxBuffer);
		HAL_UART_Receive_DMA(&huart2, &uart2RxBuffer, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		HAL_UART_Receive_DMA(&huart1, &uart1RxBuffer, 1);
	}
	if(huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_DMA(&huart2, &uart2RxBuffer, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		HAL_UART_DeInit(&huart1);
		MX_USART1_UART_Init();
		HAL_UART_Receive_DMA(&huart1, &uart1RxBuffer, 1);
	}
	if(huart->Instance == huart2.Instance)
	{
		HAL_UART_DeInit(&huart2);
		MX_USART2_UART_Init();
		HAL_UART_Receive_DMA(&huart2, &uart2RxBuffer, 1);
	}
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
	__disable_irq();
	while (1)
	{
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
