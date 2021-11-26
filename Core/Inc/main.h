/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define PID_MAX 3000
#define PID_MIN 0
    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */
    void forward(int);                       //小车前进，输入为pwm
    void backward(int);                      //小车后�??，输入为pwm
    void rotate_clockwise(int);              //小车顺时针原地转�?
    void rotate_counterclockwise(int);       //小车逆时针原地转�?
    void rotate_clockwise_plus_forward(int); //行进中拐�?
    void Goto(int, int);                     //给定两个输入x,y，走到那里去
    float pid(float);
    short Abs(short);
    int Solve_Mine_Pos(uint16_t, uint16_t, uint32_t, uint16_t, uint16_t, uint32_t, uint16_t, uint16_t, uint32_t, double *, double *);
    double fabs(double x);
    double atan2LUTif(double y, double x);
    void Sol_Car_Pos_INIT();
    void Sol_Car_Pos(double r_1, double r_2, double r_3);
    double atan2LUTif(double y, double x);
    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

    /* USER CODE BEGIN EFP */
    //float pid(pid_instance &, float);
    /* USER CODE END EFP */

    /* Private defines -----------------------------------------------------------*/
    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
