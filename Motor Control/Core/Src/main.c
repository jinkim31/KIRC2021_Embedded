/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID_form.h"
#include "HalFlash.h"
#include "../../ScueDK/scuedk_embedded/slave/inc/serial_line.h"
#include "../../ScueDK/structs/structs.h"

//#include "HALFlash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RPM_FILTER_SIZE 10
#define FL_FLASH_SIZE 20 // pidPos 6 pidVel 6 pidCur 6 MaxRPM 1 MaxTorque 1
#define RX_BUFFER_SIZE 300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




enum{
	MODE_PWM, MODE_MOTOR_INIT, MODE_PID, MODE_FLASH
};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* COMM */
uint8_t rxBuffer[RX_BUFFER_SIZE];
SerialLine serialLine;
TrackController tc;
FlipperController fc;
/* MODE */
int MODE=0;

/* cnt */
int tim6_cnt=0;
int tim7_cnt=0;
int adc_cnt=0;


int enc=0;
int enc1=0;
int enc2=0;


unsigned int adc_buffer[2];
unsigned int current_buffer[2];

//test
float Target;
float Target_Pos_FL;

float rpm_temp[RPM_FILTER_SIZE]={0,};
int rpm_filter_cnt=0;

int test=0;


/* PWM */
int FL_Duty[4] = {500,};

/* Flipper */
PID pidVel_FL0;	//LEFT
PID pidVel_FL1;	//RIGHT
PID pidVel_FL2;	//LEFT
PID pidVel_FL3;	//RIGHT

PID pidPos_FL0;	//LEFT
PID pidPos_FL1;	//RIGHT
PID pidPos_FL2;	//LEFT
PID pidPos_FL3;	//RIGHT

PID pidCur_FL0;	//LEFT
PID pidCur_FL1;	//RIGHT
PID pidCur_FL2;	//LEFT
PID pidCur_FL3;	//RIGHT

MOTOR motor_FL0;
MOTOR motor_FL1;
MOTOR motor_FL2;
MOTOR motor_FL3;

pidDebug debug_FL0;
pidDebug debug_FL1;

/* Base */
PID pidVel_BASE_L;
PID pidVel_BASE_R;
MOTOR motor_BASE_L;
MOTOR motor_BASE_R;


pidDebug debug_BASE;

/* Lead Switch */
int LS_FL[4]={0,};

/* Flash */
int Flash_Flag=0;

int Flash_Read_Data_FL0[FL_FLASH_SIZE]={0,};
int Flash_Read_Data_FL1[FL_FLASH_SIZE]={0,};
int Flash_Read_Data_FL2[FL_FLASH_SIZE]={0,};
int Flash_Read_Data_FL3[FL_FLASH_SIZE]={0,};

int Flash_Write_Data_FL0[FL_FLASH_SIZE]={0,};
int Flash_Write_Data_FL1[FL_FLASH_SIZE]={0,};
int Flash_Write_Data_FL2[FL_FLASH_SIZE]={0,};
int Flash_Write_Data_FL3[FL_FLASH_SIZE]={0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void motorInit();

void pidInit();

void setDuty(MOTOR* motor, float Target_Duty);

void setRPM(MOTOR* motor, float Target_RPM);

void RPM_filter(float rpm);

void getLeadSwitch();

void Read_FlashData();
void Set_FlashData();
void Write_FlashData();


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
  //initSerialLine(&serialLine, 1, (uint8_t*)&fc, sizeof(fc), USART3);
  initSerialLine(&serialLine, 2, (uint8_t*)&tc, sizeof(tc), USART3);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, rxBuffer);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, &USART3->DR);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, RX_BUFFER_SIZE);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableIT_IDLE(USART3);


  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7); //1ms

//  HAL_ADCEx_Calibration_Start(&hadc1, 10);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)current_buffer, 2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 2);

//  HAL_ADCEx_Calibration_Start(&hadc2, 10);
//  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)current_buffer, 2);





  //Flipper0 --> Flipper Back
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);			//PA7
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	//PA15 PB9

  //Flipper1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);			//PA6
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);	//PA8 PA9

  //Flipper2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);			//PB0
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	//PB6 PB7

  //Flipper4
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);			//PB1
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);	//PC6 PC7



//  HAL_TIM_Base_Start_IT(&htim4);


  motorInit();
  pidInit();

  TIM3->CCR1 = 500;
  TIM3->CCR2 = 500;
  TIM3->CCR3 = 500;
  TIM3->CCR4 = 500;


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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM6)    //1ms
	{
		tim6_cnt++;

//		TIM4->CCR1 = Target;
//		TIM4->CCR2 = duty2;
//		TIM4->CCR3 = duty3;


//		Get_Motor_RPM(&motor_FL0, TIM1);




//		setRPM(&motor_BASE_R, Target);
//		Get_Motor_RPM_BASE(&motor_BASE_R, TIM3);
//		PID_Control(&pidVel_BASE_R, motor_BASE_R.TargetRPM, motor_BASE_R.RPM);
//		setDuty(&motor_BASE_R, pidVel_BASE_R.nowOutput);

		switch(MODE)
		{
		case MODE_PWM:
		{
			TIM3->CCR1 = FL_Duty[0];
			TIM3->CCR2 = FL_Duty[1];
			TIM3->CCR3 = FL_Duty[2];
			TIM3->CCR4 = FL_Duty[3];

			break;
		}
		case MODE_MOTOR_INIT:
		{
			TIM3->CCR1 = 500;
			TIM3->CCR2 = 500;
			TIM3->CCR3 = 500;
			TIM3->CCR4 = 500;

			MotorData_Init(&motor_FL0, TIM2);
			MotorData_Init(&motor_FL1, TIM1);
			MotorData_Init(&motor_FL2, TIM4);
			MotorData_Init(&motor_FL3, TIM8);

			break;
		}
		case MODE_PID:
		{

			break;
		}
		case MODE_FLASH:
		{
			if(Flash_Flag == 1)
			{
				Set_FlashData();
				Write_FlashData();
				Flash_Flag = 0;
			}
			else if(Flash_Flag == 2)
			{
				Read_FlashData();
			}
			break;
		}
		}


		if(test==0)
		{
			Get_Motor_RPM(&motor_FL2, TIM2);
			RPM_filter(motor_FL2.RPM);
			PID_Control(&pidVel_FL2, pidPos_FL2.nowOutput, motor_FL2.realRPM);
			setDuty(&motor_FL2, pidVel_FL2.nowOutput);
		}

		else if(test == 1)
		{

			Get_Motor_RPM(&motor_FL2, TIM2);
		}


	}

	if (htim->Instance == TIM7)    //10ms
	{
		tim7_cnt++;
		if(test==0)
		{
			PID_Control_FL(&pidPos_FL2, Target_Pos_FL, motor_FL2.Degree);
		}

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	if(hadc->Instance == hadc1.Instance)
//	{
//		cnt_adc1++;
//	}

//	if(hadc->Instance == hadc2.Instance)
//	{
////		HAL_ADC_Start_DMA(&hadc2, (uint32_t *)current_buffer, 2);
//		cnt_adc2++;
//	}
}

//-----------------------------------------------------------------------------------------------------------

void motorInit(){
	motor_FL0.MotorNum = 0;
	motor_FL1.MotorNum = 1;
	motor_FL2.MotorNum = 2;
	motor_FL3.MotorNum = 3;

	motor_BASE_L.MotorNum = 4;
	motor_BASE_R.MotorNum = 5;
	motor_BASE_R.maxRPM = 50;

}

void pidInit(){
	pidVel_BASE_R.underOfPoint=200;
	pidVel_BASE_R.outputLimit=200;
	pidVel_BASE_R.errorSumLimit=1000;
	pidVel_BASE_R.kP=10000;
	pidVel_BASE_R.kI=10;
	pidVel_BASE_R.kD=0;

	pidVel_BASE_L.underOfPoint=200;
	pidVel_BASE_L.outputLimit=200;
	pidVel_BASE_L.errorSumLimit=1000;
	pidVel_BASE_L.kP=10000;
	pidVel_BASE_L.kI=10;
	pidVel_BASE_L.kD=0;

	pidVel_FL2.underOfPoint=100;
	pidVel_FL2.outputLimit=200;
	pidVel_FL2.errorSumLimit=3000;
	pidVel_FL2.kP=200;
	pidVel_FL2.kI=10;
	pidVel_FL2.kD=0;

	pidPos_FL2.underOfPoint=10000;
	pidPos_FL2.outputLimit=20;
	pidPos_FL2.errorSumLimit=5000;
	pidPos_FL2.kP=-2000;
	pidPos_FL2.kI=-3;
	pidPos_FL2.kD=0;
}

void setDuty(MOTOR* motor, float Target_Duty){

	switch(motor->MotorNum){
	case 0:
	{
		TIM4->CCR1 = Target_Duty;
		break;
	}
	case 1:
	{
		TIM4->CCR1 = Target_Duty;
		break;
	}
	case 2:
	{
		Target_Duty *= -1;
		Target_Duty += 500;
		TIM4->CCR3 = Target_Duty;
		break;
	}
	case 3:
	{
		TIM4->CCR1 = Target_Duty;
		break;
	}
	case 4:
	{
		TIM4->CCR1 = Target_Duty;
		break;
	}
	case 5:
	{
		Target_Duty *= -1;
		Target_Duty += 500;
		TIM4->CCR1 = Target_Duty;
		break;
	}
	}
}

void setRPM(MOTOR* motor, float Target_RPM){
	motor->TargetRPM = Target_RPM;
}

void RPM_filter(float rpm){

	float sum=0;

	rpm_temp[rpm_filter_cnt] = rpm;
	rpm_filter_cnt++;

	if(rpm_filter_cnt == RPM_FILTER_SIZE)
	{
		for(int i=0;i<10;i++)
		{
			sum += rpm_temp[i];
		}
		motor_FL2.realRPM = sum/RPM_FILTER_SIZE;
		rpm_filter_cnt=0;
	}
}

void getLeadSwitch(){
	LS_FL[0] = HAL_GPIO_ReadPin(LeadSwitch_FL0_GPIO_Port, LeadSwitch_FL0_Pin);
	LS_FL[1] = HAL_GPIO_ReadPin(LeadSwitch_FL1_GPIO_Port, LeadSwitch_FL1_Pin);
	LS_FL[2] = HAL_GPIO_ReadPin(LeadSwitch_FL2_GPIO_Port, LeadSwitch_FL2_Pin);
	LS_FL[3] = HAL_GPIO_ReadPin(LeadSwitch_FL3_GPIO_Port, LeadSwitch_FL3_Pin);
}

void Read_FlashData()
{
  int ByteCnt=0;
  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Read_Data_FL0[i] = Flash_Read(Init_Add + ByteCnt);
	  ByteCnt += 4;
  }

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Read_Data_FL1[i] = Flash_Read(Init_Add + ByteCnt);
	  ByteCnt += 4;
  }

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Read_Data_FL2[i] = Flash_Read(Init_Add + ByteCnt);
	  ByteCnt += 4;
  }

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Read_Data_FL3[i] = Flash_Read(Init_Add + ByteCnt);
	  ByteCnt += 4;
  }
}

void Set_FlashData()
{
  Flash_Write_Data_FL0[0] = pidPos_FL0.kP* (-1);
  Flash_Write_Data_FL0[1] = pidPos_FL0.kI* (-1);
  Flash_Write_Data_FL0[2] = pidPos_FL0.kD* (-1);
  Flash_Write_Data_FL0[3] = pidPos_FL0.errorSumLimit;
  Flash_Write_Data_FL0[4] = pidPos_FL0.outputLimit;
  Flash_Write_Data_FL0[5] = pidPos_FL0.underOfPoint;

  Flash_Write_Data_FL0[6] = pidVel_FL0.kP;
  Flash_Write_Data_FL0[7] = pidVel_FL0.kI;
  Flash_Write_Data_FL0[8] = pidVel_FL0.kD;
  Flash_Write_Data_FL0[9] = pidVel_FL0.errorSumLimit;
  Flash_Write_Data_FL0[10] = pidVel_FL0.outputLimit;
  Flash_Write_Data_FL0[11] = pidVel_FL0.underOfPoint;

  Flash_Write_Data_FL0[12] = pidCur_FL0.kP;
  Flash_Write_Data_FL0[13] = pidCur_FL0.kI;
  Flash_Write_Data_FL0[14] = pidCur_FL0.kD;
  Flash_Write_Data_FL0[15] = pidCur_FL0.errorSumLimit;
  Flash_Write_Data_FL0[16] = pidCur_FL0.outputLimit;
  Flash_Write_Data_FL0[17] = pidCur_FL0.underOfPoint;

  Flash_Write_Data_FL0[18] = pidPos_FL0.outputLimit;	//maxRPM
  Flash_Write_Data_FL0[19] = pidVel_FL0.outputLimit;	//maxCUR

//-------------------------------------------------------------------
  Flash_Write_Data_FL1[0] = pidPos_FL1.kP* (-1);
  Flash_Write_Data_FL1[1] = pidPos_FL1.kI* (-1);
  Flash_Write_Data_FL1[2] = pidPos_FL1.kD* (-1);
  Flash_Write_Data_FL1[3] = pidPos_FL1.errorSumLimit;
  Flash_Write_Data_FL1[4] = pidPos_FL1.outputLimit;
  Flash_Write_Data_FL1[5] = pidPos_FL1.underOfPoint;

  Flash_Write_Data_FL1[6] = pidVel_FL1.kP;
  Flash_Write_Data_FL1[7] = pidVel_FL1.kI;
  Flash_Write_Data_FL1[8] = pidVel_FL1.kD;
  Flash_Write_Data_FL1[9] = pidVel_FL1.errorSumLimit;
  Flash_Write_Data_FL1[10] = pidVel_FL1.outputLimit;
  Flash_Write_Data_FL1[11] = pidVel_FL1.underOfPoint;

  Flash_Write_Data_FL1[12] = pidCur_FL1.kP;
  Flash_Write_Data_FL1[13] = pidCur_FL1.kI;
  Flash_Write_Data_FL1[14] = pidCur_FL1.kD;
  Flash_Write_Data_FL1[15] = pidCur_FL1.errorSumLimit;
  Flash_Write_Data_FL1[16] = pidCur_FL1.outputLimit;
  Flash_Write_Data_FL1[17] = pidCur_FL1.underOfPoint;

  Flash_Write_Data_FL1[18] = pidPos_FL1.outputLimit;	//maxRPM
  Flash_Write_Data_FL1[19] = pidVel_FL1.outputLimit;	//maxCUR

  //-------------------------------------------------------------------
  Flash_Write_Data_FL2[0] = pidPos_FL2.kP* (-1);
  Flash_Write_Data_FL2[1] = pidPos_FL2.kI* (-1);
  Flash_Write_Data_FL2[2] = pidPos_FL2.kD* (-1);
  Flash_Write_Data_FL2[3] = pidPos_FL2.errorSumLimit;
  Flash_Write_Data_FL2[4] = pidPos_FL2.outputLimit;
  Flash_Write_Data_FL2[5] = pidPos_FL2.underOfPoint;

  Flash_Write_Data_FL2[6] = pidVel_FL2.kP;
  Flash_Write_Data_FL2[7] = pidVel_FL2.kI;
  Flash_Write_Data_FL2[8] = pidVel_FL2.kD;
  Flash_Write_Data_FL2[9] = pidVel_FL2.errorSumLimit;
  Flash_Write_Data_FL2[10] = pidVel_FL2.outputLimit;
  Flash_Write_Data_FL2[11] = pidVel_FL2.underOfPoint;

  Flash_Write_Data_FL2[12] = pidCur_FL2.kP;
  Flash_Write_Data_FL2[13] = pidCur_FL2.kI;
  Flash_Write_Data_FL2[14] = pidCur_FL2.kD;
  Flash_Write_Data_FL2[15] = pidCur_FL2.errorSumLimit;
  Flash_Write_Data_FL2[16] = pidCur_FL2.outputLimit;
  Flash_Write_Data_FL2[17] = pidCur_FL2.underOfPoint;

  Flash_Write_Data_FL2[18] = pidPos_FL2.outputLimit;	//maxRPM
  Flash_Write_Data_FL2[19] = pidVel_FL2.outputLimit;	//maxCUR

  //-------------------------------------------------------------------
  Flash_Write_Data_FL3[0] = pidPos_FL3.kP* (-1);
  Flash_Write_Data_FL3[1] = pidPos_FL3.kI* (-1);
  Flash_Write_Data_FL3[2] = pidPos_FL3.kD* (-1);
  Flash_Write_Data_FL3[3] = pidPos_FL3.errorSumLimit;
  Flash_Write_Data_FL3[4] = pidPos_FL3.outputLimit;
  Flash_Write_Data_FL3[5] = pidPos_FL3.underOfPoint;

  Flash_Write_Data_FL3[6] = pidVel_FL3.kP;
  Flash_Write_Data_FL3[7] = pidVel_FL3.kI;
  Flash_Write_Data_FL3[8] = pidVel_FL3.kD;
  Flash_Write_Data_FL3[9] = pidVel_FL3.errorSumLimit;
  Flash_Write_Data_FL3[10] = pidVel_FL3.outputLimit;
  Flash_Write_Data_FL3[11] = pidVel_FL3.underOfPoint;

  Flash_Write_Data_FL3[12] = pidCur_FL3.kP;
  Flash_Write_Data_FL3[13] = pidCur_FL3.kI;
  Flash_Write_Data_FL3[14] = pidCur_FL3.kD;
  Flash_Write_Data_FL3[15] = pidCur_FL3.errorSumLimit;
  Flash_Write_Data_FL3[16] = pidCur_FL3.outputLimit;
  Flash_Write_Data_FL3[17] = pidCur_FL3.underOfPoint;

  Flash_Write_Data_FL3[18] = pidPos_FL3.outputLimit;	//maxRPM
  Flash_Write_Data_FL3[19] = pidVel_FL3.outputLimit;	//maxCUR


}

void Write_FlashData()
{
  int Check_Cnt=0;
  int Byte_Cnt=0;

  Flash_Erase(FLASH_SECTOR_5);

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
    Flash_Write(Init_Add + Byte_Cnt, Flash_Write_Data_FL0[i]);
    Byte_Cnt += 4;
    HAL_FLASH_Lock();
  }

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Write(Init_Add + Byte_Cnt, Flash_Write_Data_FL1[i]);
	  Byte_Cnt += 4;
	  HAL_FLASH_Lock();
  }

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Write(Init_Add + Byte_Cnt, Flash_Write_Data_FL2[i]);
	  Byte_Cnt += 4;
	  HAL_FLASH_Lock();
  }

  for(int i=0;i<FL_FLASH_SIZE;i++)
  {
	  Flash_Write(Init_Add + Byte_Cnt, Flash_Write_Data_FL3[i]);
	  Byte_Cnt += 4;
	  HAL_FLASH_Lock();
  }

}

void Init_FlashData(){
	pidPos_FL0.kP=Flash_Read_Data_FL0[0]* (-1);
	pidPos_FL0.kI=Flash_Read_Data_FL0[1]* (-1);
	pidPos_FL0.kD=Flash_Read_Data_FL0[2]* (-1);
	pidPos_FL0.errorSumLimit=Flash_Read_Data_FL0[3];
	pidPos_FL0.outputLimit=Flash_Read_Data_FL0[4];
	pidPos_FL0.underOfPoint=Flash_Read_Data_FL0[5];

	pidVel_FL0.kP=Flash_Read_Data_FL0[6];
	pidVel_FL0.kI=Flash_Read_Data_FL0[7];
	pidVel_FL0.kD=Flash_Read_Data_FL0[8];
	pidVel_FL0.errorSumLimit=Flash_Read_Data_FL0[9];
	pidVel_FL0.outputLimit=Flash_Read_Data_FL0[10];
	pidVel_FL0.underOfPoint=Flash_Read_Data_FL0[11];

	pidCur_FL0.kP=Flash_Read_Data_FL0[12];
	pidCur_FL0.kI=Flash_Read_Data_FL0[13];
	pidCur_FL0.kD=Flash_Read_Data_FL0[14];
	pidCur_FL0.errorSumLimit=Flash_Read_Data_FL0[15];
	pidCur_FL0.outputLimit=Flash_Read_Data_FL0[16];
	pidCur_FL0.underOfPoint=Flash_Read_Data_FL0[17];

	pidPos_FL0.outputLimit=Flash_Read_Data_FL0[18];		//maxRPM
	pidVel_FL0.outputLimit==Flash_Read_Data_FL0[19];	//maxCUR

//-------------------------------------------------------------------
	pidPos_FL1.kP=Flash_Read_Data_FL1[0]* (-1);
	pidPos_FL1.kI=Flash_Read_Data_FL1[1]* (-1);
	pidPos_FL1.kD=Flash_Read_Data_FL1[2]* (-1);
	pidPos_FL1.errorSumLimit=Flash_Read_Data_FL1[3];
	pidPos_FL1.outputLimit=Flash_Read_Data_FL1[4];
	pidPos_FL1.underOfPoint=Flash_Read_Data_FL1[5];

	pidVel_FL1.kP=Flash_Read_Data_FL1[6];
	pidVel_FL1.kI=Flash_Read_Data_FL1[7];
	pidVel_FL1.kD=Flash_Read_Data_FL1[8];
	pidVel_FL1.errorSumLimit=Flash_Read_Data_FL1[9];
	pidVel_FL1.outputLimit=Flash_Read_Data_FL1[10];
	pidVel_FL1.underOfPoint=Flash_Read_Data_FL1[11];

	pidCur_FL1.kP=Flash_Read_Data_FL1[12];
	pidCur_FL1.kI=Flash_Read_Data_FL1[13];
	pidCur_FL1.kD=Flash_Read_Data_FL1[14];
	pidCur_FL1.errorSumLimit=Flash_Read_Data_FL1[15];
	pidCur_FL1.outputLimit=Flash_Read_Data_FL1[16];
	pidCur_FL1.underOfPoint=Flash_Read_Data_FL1[17];

	pidPos_FL1.outputLimit=Flash_Read_Data_FL1[18];		//maxRPM
	pidVel_FL1.outputLimit==Flash_Read_Data_FL1[19];	//maxCUR

	//-------------------------------------------------------------------
	pidPos_FL2.kP=Flash_Read_Data_FL2[0]* (-1);
	pidPos_FL2.kI=Flash_Read_Data_FL2[1]* (-1);
	pidPos_FL2.kD=Flash_Read_Data_FL2[2]* (-1);
	pidPos_FL2.errorSumLimit=Flash_Read_Data_FL2[3];
	pidPos_FL2.outputLimit=Flash_Read_Data_FL2[4];
	pidPos_FL2.underOfPoint=Flash_Read_Data_FL2[5];

	pidVel_FL2.kP=Flash_Read_Data_FL2[6];
	pidVel_FL2.kI=Flash_Read_Data_FL2[7];
	pidVel_FL2.kD=Flash_Read_Data_FL2[8];
	pidVel_FL2.errorSumLimit=Flash_Read_Data_FL2[9];
	pidVel_FL2.outputLimit=Flash_Read_Data_FL2[10];
	pidVel_FL2.underOfPoint=Flash_Read_Data_FL2[11];

	pidCur_FL2.kP=Flash_Read_Data_FL2[12];
	pidCur_FL2.kI=Flash_Read_Data_FL2[13];
	pidCur_FL2.kD=Flash_Read_Data_FL2[14];
	pidCur_FL2.errorSumLimit=Flash_Read_Data_FL2[15];
	pidCur_FL2.outputLimit=Flash_Read_Data_FL2[16];
	pidCur_FL2.underOfPoint=Flash_Read_Data_FL2[17];

	pidPos_FL2.outputLimit=Flash_Read_Data_FL2[18];		//maxRPM
	pidVel_FL2.outputLimit==Flash_Read_Data_FL2[19];	//maxCUR

	//-------------------------------------------------------------------
	pidPos_FL3.kP=Flash_Read_Data_FL3[0]* (-1);
	pidPos_FL3.kI=Flash_Read_Data_FL3[1]* (-1);
	pidPos_FL3.kD=Flash_Read_Data_FL3[2]* (-1);
	pidPos_FL3.errorSumLimit=Flash_Read_Data_FL3[3];
	pidPos_FL3.outputLimit=Flash_Read_Data_FL3[4];
	pidPos_FL3.underOfPoint=Flash_Read_Data_FL3[5];

	pidVel_FL3.kP=Flash_Read_Data_FL3[6];
	pidVel_FL3.kI=Flash_Read_Data_FL3[7];
	pidVel_FL3.kD=Flash_Read_Data_FL3[8];
	pidVel_FL3.errorSumLimit=Flash_Read_Data_FL3[9];
	pidVel_FL3.outputLimit=Flash_Read_Data_FL3[10];
	pidVel_FL3.underOfPoint=Flash_Read_Data_FL3[11];

	pidCur_FL3.kP=Flash_Read_Data_FL3[12];
	pidCur_FL3.kI=Flash_Read_Data_FL3[13];
	pidCur_FL3.kD=Flash_Read_Data_FL3[14];
	pidCur_FL3.errorSumLimit=Flash_Read_Data_FL3[15];
	pidCur_FL3.outputLimit=Flash_Read_Data_FL3[16];
	pidCur_FL3.underOfPoint=Flash_Read_Data_FL3[17];

	pidPos_FL3.outputLimit=Flash_Read_Data_FL3[18];		//maxRPM
	pidVel_FL3.outputLimit==Flash_Read_Data_FL3[19];	//maxCUR

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
