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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "zigbee.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy62.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PID_MAX 3000
#define PID_MIN 0
float set_speed = 10.0;
int count_to_ten = 0;
volatile float distance1 = 0.0, distance2 = 0.0, distance3 = 0.0, distance4 = 0.0;
extern uint8_t jy62Receive[JY62_MESSAGE_LENGTH]; //实时记录收到的信�?
extern uint8_t jy62Message[JY62_MESSAGE_LENGTH]; //确认无误后用于解码的信息
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const float Kp = 300.0;
const float Ki = -118;
const float Kd = 50;
float state[3] = {0.0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float pid(float in)
{
    float out;

    // y[n] = A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
    out = ((Kp + Ki + Kd) * in) +
          ((-Ki - 2 * Kd) * state[0]) + (Kd * state[1]);

    state[1] = state[0];
    state[0] = in;

    if (out >= PID_MAX)
        state[2] = PID_MAX;
    else if (out <= PID_MIN)
        state[2] = PID_MIN;
    else
        state[2] = out;
    return (state[2]);
}

short Abs(short in)
{
    return in >= 0 ? in : -in;
}

void rotate()
{
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_UART_Receive_DMA(&huart2, jy62Receive, JY62_MESSAGE_LENGTH);

    if (htim->Instance == TIM1)
    {
        count_to_ten++;
        short Count[4] = {
            Abs(__HAL_TIM_GetCounter(&htim2)),
            Abs(__HAL_TIM_GetCounter(&htim3)),
            Abs(__HAL_TIM_GetCounter(&htim4)),
            Abs(__HAL_TIM_GetCounter(&htim5))};

        __HAL_TIM_SetCounter(&htim2, 0);
        __HAL_TIM_SetCounter(&htim3, 0);
        __HAL_TIM_SetCounter(&htim4, 0);
        __HAL_TIM_SetCounter(&htim5, 0);
        float speed1, speed2, speed3, speed4;
        speed1 = (float)Count[0] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s
        speed2 = (float)Count[1] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s
        speed3 = (float)Count[2] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s
        speed4 = (float)Count[3] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s

        //距离+=速度*定时器中断周周期
        if (speed1 < 300)
            distance1 = speed1 / 500.0 + distance1;
        if (speed2 < 300)
            distance2 = speed2 / 500.0 + distance2;
        if (speed3 < 300)
            distance3 = speed3 / 500.0 + distance3;
        if (speed4 < 300)
            distance4 = speed4 / 500.0 + distance4;
        float ave_distance = (distance1 + distance2 + distance3 + distance4) / 4;

        // if (count_to_ten == 10)
        // {
        //     u1_printf("%f,%f,%f,%f", speed1, speed2 / 100.0, speed3 / 100.0, speed4 / 100.0);
        //     u1_printf("%f,%f,%f,%f,%f\n", distance1, distance2, distance3, distance4, GetYaw());
        //     u1_printf("%f,%f,%f,%f\n", speed1, speed2, speed3, speed4);
        //     u1_printf("%f\n", GetYaw());
        //     count_to_ten = 0;
        // }

        //移动距离超过0.5m，顺时针�?120°,如果没有，保持直�?
        if (ave_distance >= 30)
        {
            float sset_speed1 = 14;
            float sset_speed2 = 14;
            float sset_speed3 = 14;
            float sset_speed4 = 14;

            float ppwm1 = pid(sset_speed1 - speed1);
            float ppwm2 = pid(sset_speed2 - speed2);
            float ppwm3 = pid(sset_speed3 - speed3);
            float ppwm4 = pid(sset_speed4 - speed4);

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, ppwm1);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, ppwm2);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, ppwm3);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, ppwm4);
        }
        else
        {
            // float k = 1;
            float pwm1 = pid(set_speed - speed1);
            float pwm2 = pid(set_speed - speed2);
            float pwm3 = pid(set_speed - speed3);
            float pwm4 = pid(set_speed - speed4);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm1);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm3);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm2);
            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwm4);
        }

        // 如果转向完成，所有数据归�?
        if (55 < GetYaw() && GetYaw() < 65)
        {
            distance1 = 0;
            distance2 = 0;
            distance3 = 0;
            distance4 = 0;
            ave_distance = 0;
            InitAngle();
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
        //解算到对方信标的距离dis_xinbiao和到对方信标的角度angle_xinbiao

        //计算当前走过路程travel
        count_to_ten++;
        short Count[4] = {
            Abs(__HAL_TIM_GetCounter(&htim2)),
            Abs(__HAL_TIM_GetCounter(&htim3)),
            Abs(__HAL_TIM_GetCounter(&htim4)),
            Abs(__HAL_TIM_GetCounter(&htim5))};

        __HAL_TIM_SetCounter(&htim2, 0);
        __HAL_TIM_SetCounter(&htim3, 0);
        __HAL_TIM_SetCounter(&htim4, 0);
        __HAL_TIM_SetCounter(&htim5, 0);
        float speed1, speed2, speed3, speed4;
        speed1 = (float)Count[0] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s
        speed2 = (float)Count[1] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s
        speed3 = (float)Count[2] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s
        speed4 = (float)Count[3] / 4000.0 * 3.142 * 6.5 * 1000; //cm/s

        if (speed1 < 300)
            distance1 = speed1 / 500.0 + distance1;
        if (speed2 < 300)
            distance2 = speed2 / 500.0 + distance2;
        if (speed3 < 300)
            distance3 = speed3 / 500.0 + distance3;
        if (speed4 < 300)
            distance4 = speed4 / 500.0 + distance4;
        float travel = (distance1 + distance2 + distance3 + distance4) / 4;

        //小车出发
        // if (flag == 0) //没有遇到障碍物的状�??
        if (1) //初赛没有障碍
        {
            if (angle_obj - GetYaw() > -5 && angle_obj - GetYaw() < 5 && dis_obj > 2 && dis_xinbiao > 10)
            {
                //直行
            }
            else if (dit_obj > 5)
            {
                if (dis_xinbiao > 10)
                {
                    rotate();
                }
                else
                    flag = 1;
            }
            else
            {
                //如果还有资源，解算目标资源的坐标(x,y)，到目标资源的距离dis_obj和角度angle_obj
                //如果资源取完了，就上仓库坐标，相应目标点变为仓库坐标，相应距离和角度同理
            }
        }
        else //遇到障碍物的状�??
        {
            float angle_temp = GetYaw() + (GetYaw() - angle_xinbiao) / fabs(GetYaw() - angle_xinbiao) * 90;
            //朝障碍物反方向转90�?
            if (GetYaw() - angle_temp < 5 && GetYaw() - angle_temp() > -5)
            {
                if (travel < 10) //转完以后�?10cm
                {
                    //直行
                }
                else
                    flag = 0;
            }
            else
            {
                rotate();
                travel = 0;
            }
        }
    }
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
    jy62_Init(&huart2);
    SetBaud(115200);
    SetHorizontal();
    InitAngle();
    Calibrate();
    SleepOrAwake();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
    zigbee_Init(&huart1);
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
    jy62_Init(&huart2);
    void jy62_Init(UART_HandleTypeDef * huart);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
