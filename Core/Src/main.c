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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy62.h"
#include "zigbee.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

float set_speed = 10.0;
int count_to_ten = 0;
volatile float distance1 = 0.0, distance2 = 0.0, distance3 = 0.0, distance4 = 0.0;
extern uint8_t jy62Receive[JY62_MESSAGE_LENGTH];
extern uint8_t jy62Message[JY62_MESSAGE_LENGTH];

float angle_obj, dis_obj;
float obj_x, obj_y, car_x, car_y;

//////////////////////////////////////Sol_Car_Pos函数变量定义///////////////////////////////////////////
int beacon_Pos[6];              //三个信标坐标，依次存放x_1、y_1、x_2、y_2、x_3、y_3
int car_Pos[2];                 //第二回合小车当前坐标
double beacon_determinant=0.0;  //计算中间变量
//////////////////////////////////////Sol_Car_Pos函数定义结束///////////////////////////////////////////
#define forward_speed 1500
#define rotate_speed 800
#define angle_err 5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//////////////////////////////////////Solve_Mine_Pos函数宏定义///////////////////////////////////////////
#define A_COFFIENT 1000000000.0 //金矿强度系数A
#define POS_ERR_TOL 4           //金矿位置计算结果偏离场地的最大误差
//////////////////////////////////////Solve_Mine_Pos宏定义结束///////////////////////////////////////////
////////////////////////////////////////atan2LUTif函数宏定义/////////////////////////////////////////////
#define M_PI_2 1.5707963
#define M_PI 3.141592654
#define M_PI_4_P_0273	1.05839816339744830962 //M_PI/4 + 0.273 
const double ATAN_LUT[256] = {0.0000000000,0.0039215485,0.0078429764,0.0117641631,0.0156849881,0.0196053309,
                              0.0235250710,0.0274440882,0.0313622624,0.0352794736,0.0391956019,0.0431105278,
                              0.0470241318,0.0509362949,0.0548468980,0.0587558227,0.0626629506,0.0665681638,
                              0.0704713446,0.0743723758,0.0782711405,0.0821675224,0.0860614053,0.0899526737,
                              0.0938412126,0.0977269074,0.1016096438,0.1054893085,0.1093657884,0.1132389710,
                              0.1171087446,0.1209749978,0.1248376255,0.1286965013,0.1325515323,0.1364026044,
                              0.1402496096,0.1440924408,0.1479309912,0.1517651553,0.1555948280,0.1594199049,
                              0.1632402828,0.1670558588,0.1708665312,0.1746721990,0.1784727620,0.1822681208,
                              0.1860581771,0.1898428334,0.1936219929,0.1973955598,0.2011634395,0.2049255380,
                              0.2086817623,0.2124320205,0.2161762215,0.2199142752,0.2236460927,0.2273715857,
                              0.2310906672,0.2348032511,0.2385092525,0.2422085871,0.2459011721,0.2495869254,
                              0.2532657662,0.2569376146,0.2606023917,0.2642600199,0.2679104224,0.2715535237,
                              0.2751892491,0.2788175253,0.2824382800,0.2860514417,0.2896569404,0.2932547070,
                              0.2968446734,0.3004267728,0.3040009393,0.3075671084,0.3111252164,0.3146752558,
                              0.3182170002,0.3217505544,0.3252758042,0.3287926915,0.3323011594,0.3358011520,
                              0.3392926145,0.3427754932,0.3462497357,0.3497152904,0.3531721069,0.3566201360,
                              0.3600593294,0.3634896400,0.3669110217,0.3703234297,0.3737268255,0.3771211497,
                              0.3805063771,0.3838824615,0.3872493632,0.3906070437,0.3939554653,0.3972945915,
                              0.4006243869,0.4039448169,0.4072558481,0.4105574480,0.4138495853,0.4171322295,
                              0.4204053512,0.4236689219,0.4269229141,0.4301673014,0.4334020581,0.4366271598,
                              0.4398425828,0.4430483044,0.4462443029,0.4494305575,0.4526070482,0.4557737560,
                              0.4589306629,0.4620777516,0.4652150058,0.4683424102,0.4714599501,0.4745676117,
                              0.4776653824,0.4807532499,0.4838312032,0.4868992318,0.4899573263,0.4930054778,
                              0.4960436784,0.4990719209,0.5020901990,0.5050985071,0.5080968402,0.5110851942,
                              0.5140635659,0.5170319525,0.5199903521,0.5229387636,0.5258771863,0.5288056206,
                              0.5317240673,0.5346325278,0.5375310045,0.5404195003,0.5432980185,0.5461665634,
                              0.5490251398,0.5518737530,0.5547124091,0.5575411147,0.5603598769,0.5631687036,
                              0.5659676030,0.5687565842,0.5715356566,0.5743048302,0.5770641155,0.5798135236,
                              0.5825530662,0.5852827553,0.5880026035,0.5907126240,0.5934128303,0.5961032364,
                              0.5987838570,0.6014547069,0.6041158015,0.6067671569,0.6094087892,0.6120407151,
                              0.6146629519,0.6172755171,0.6198784285,0.6224717045,0.6250553640,0.6276294258,
                              0.6301939095,0.6327488350,0.6352942223,0.6378300921,0.6403564651,0.6428733625,
                              0.6453808058,0.6478788169,0.6503674179,0.6528466311,0.6553164793,0.6577769856,
                              0.6602281731,0.6626700655,0.6651026865,0.6675260602,0.6699402110,0.6723451634,
                              0.6747409422,0.6771275725,0.6795050796,0.6818734889,0.6842328261,0.6865831172,
                              0.6889243882,0.6912566655,0.6935799756,0.6958943450,0.6981998008,0.7004963699,
                              0.7027840796,0.7050629571,0.7073330300,0.7095943260,0.7118468729,0.7140906986,
                              0.7163258312,0.7185522990,0.7207701302,0.7229793534,0.7251799971,0.7273720900,
                              0.7295556609,0.7317307387,0.7338973524,0.7360555311,0.7382053040,0.7403467003,
                              0.7424797493,0.7446044805,0.7467209234,0.7488291075,0.7509290624,0.7530208178,
                              0.7551044035,0.7571798492,0.7592471847,0.7613064400,0.7633576449,0.7654008294,
                              0.7674360235,0.7694632573,0.7714825607,0.7734939638,0.7754974968,0.7774931897,
                              0.7794810727,0.7814611759,0.7834335294,0.7853981634};
////////////////////////////////////////atan2LUTif宏定义结束/////////////////////////////////////////////

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_UART_Receive_DMA(&huart2, jy62Receive, JY62_MESSAGE_LENGTH);

    if (htim->Instance == TIM1)
    {
        float angle_car = GetYaw();
        if (angle_obj >= angle_err && angle_obj <= 360.0 - angle_err)
        {
            if (angle_car >= angle_obj - angle_err && angle_car <= angle_obj + angle_err && dis_obj > 2)
            {
                forward(forward_speed);
            }
            else
            {
                rotate_clockwise(rotate_speed);
            }
        }
        else
        {
            if (angle_obj < angle_err)
            {
                if ((angle_car >= angle_obj - angle_err + 360.0 || angle_car <= angle_obj + angle_err) && dis_obj > 2)
                {
                    forward(forward_speed);
                }
                else
                {
                    rotate_clockwise(rotate_speed);
                }
            }
            else if (angle_obj > 360 - angle_err)
            {
                if ((angle_car <= angle_obj + angle_err - 360.0 || angle_car >= angle_obj - angle_err) && dis_obj > 2)
                {
                    forward(forward_speed);
                }
                else
                {
                    rotate_clockwise(rotate_speed);
                }
            }
        }

        //when head and the resourse(obj) are in the same half plane
        // if (angle_car > qiuyu360(angle_obj - 90) && angel_car < qiuyu360(angle_obj + 90))
        // {
        //     if (angle_car >= qiuyu360(angle_obj - angle_err) && angle_car <= qiuyu360(angle_obj + angle_err) && dis_obj > 2) //head towards resourse
        //         forward(forward_speed);
        //     else if (dis_obj > 2 && angle_car < (angle_obj - angle_err + 360) % 360)
        //         rotate_clockwise(rotate_speed);
        //     else if (dis_obj > 2 && angle_car > (angle_obj + angle_err + 360) % 360)
        //         rotate_counterclockwise(rotate_speed);
        // }
        // //when tail and the resourse(obj) are in the same half plane
        // else
        // {
        //     if (angle_abs <= 185.0 && angle_abs >= 175.0 && dis_obj > 2) //tail towards resourse
        //         backward(forward_speed);
        //     else if (dis_obj > 2 && angle_abs > 185.0)
        //         rotate_counterclockwise(rotate_speed);
        //     else if (dis_obj > 2 && angle_abs < 175.0)
        //         rotate_clockwise(rotate_speed);
        // }
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
float pid(float in)
{
    float out;
    // y[n] = A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
    out = ((Kp + Ki + Kd) * in) + ((-Ki - 2 * Kd) * state[0]) + (Kd * state[1]);
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

float qiuyu360(float in)
{
    while (in > 360)
        in -= 360.0;
    while (in < 0)
        in += 360.0;
    return in;
}

void forward(int pwm)
{
    //change rotate direction of wheel
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //give pwm output
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwm);
}

void backward(int pwm)
{
    //change rotate direction of wheel
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    //give pwm output
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwm);
}

void rotate_counterclockwise(int pwm)
{
    //change rotate direction of wheel
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    //give pwm output
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwm);
}

void rotate_clockwise(int pwm)
{
    //change rotate direction of wheel
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //give pwm output
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwm);
}
// double off_set_angle(uint16_t car_x,uint16_t car_y,double des_x,double des_y,double yaw){
//     /*
//     入口参数：小车xy坐标，目标点xy坐标，陀螺仪偏航角yaw
//     计算偏差角 函数。
//     返回值为偏差角度的绝对值，单位为度。
//     */
//     double angle=atan2(des_y-car_y,des_x-car_x);
//     double ans=yaw-angle;
//     ans=(ans<0)?(ans+360):ans;
//     ans=(ans>360)?(ans-360):ans;
//     return ans*180/pi;
// }
double fabs(double x){
    //双精度浮点数求绝对值函数
    return (x>0)?x:(-x);
}
double atan2LUTif(double y,double x){
    //反正切函数实现。误差不大于0.5°。入口参数：两点间y、x坐标的差值。
    double absx, absy, val;
    if (x == 0 && y == 0){
        return 0;
    }
    absy = fabs(y);
    absx = fabs(x);
    if (absy - absx == absy){
        return y < 0 ? -M_PI_2 : M_PI_2;
    }
    if (absx - absy == absx){
        val = 0.0;
    }
    else{
        if (y>0){
            if (absx > absy)
                val = ATAN_LUT[(int)(255*absy/absx)];//1st octant
            else
                val = M_PI_2 - ATAN_LUT[(int)(255*absx/absy)];//2nd octant
            val = x < 0 ? (M_PI - val) : val; //3-4th octants from 2-1
        }
        else{
            if (absx > absy)
                val = -ATAN_LUT[(int)(255*absy/absx)];//8th octant
            else
                val = -M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//7th octant
            val = x < 0 ? -M_PI - val : val; //5-6th octants from 8-7
        }
    }
  return val*180/M_PI;
}
int Solve_Mine_Pos(uint16_t xx_1, uint16_t yy_1, uint32_t EE_1, uint16_t xx_2, uint16_t yy_2, uint32_t EE_2, uint16_t xx_3, uint16_t yy_3, uint32_t EE_3, double *coordinate)
{
    /*
    入口参数：xx_i,yy_i,EE_i，(i=1,2,3)为三个不同点的坐标&场强。double *coordinate是用于存放计算结果（金矿坐标）的double数组
    此函数根据三组坐标&场强数据进行解算，将得到的结果存储在coordinate数组中。在较坏的情况下，误差在3cm以内。
    返回值为0或1。case 1：计算无明显异常;case 0：三个点数据量不够，或是计算结果偏出场地以外。需重新计算。
    */
    /*
    计算公式为：
    y=(A*((xx_2-xx_3)/EE_1+(xx_3-xx_1)/EE_2+(xx_1-xx_2)/EE_3)+(xx_1-xx_2)*(xx_2-xx_3)*(xx_3-xx_1)+yy_1*yy_1*(xx_3-xx_2)+yy_2*yy_2*(xx_1-xx_3)+yy_3*yy_3*(xx_2-xx_1))/2/(xx_1*yy_2-xx_2*yy_1+xx_3*yy_1-xx_1*yy_3+xx_2*yy_3-xx_3*yy_2);
    x=(xx_1+xx_2-((xx_2-xx_3)*(A/EE_1-A/EE_2)-(yy_1-yy_2)*(xx_2-xx_3)*(yy_1+yy_2-2*y))/(xx_1-xx_2)/(xx_2-xx_3))/2;
    此公式在matlab上运算正常，精度满足要求
    */
    double determinant = xx_1 * yy_2 - xx_2 * yy_1 + xx_3 * yy_1 - xx_1 * yy_3 + xx_2 * yy_3 - xx_3 * yy_2;
    if (!determinant) //??????
        return 0;
    double y = (((double)(xx_2 - xx_3) * (A_COFFIENT / (double)EE_1) + (double)(xx_3 - xx_1) * (A_COFFIENT / (double)EE_2) + (double)(xx_1 - xx_2) * (double)(A_COFFIENT / (double)EE_3)) / determinant + (xx_1 - xx_2) * (double)(xx_2 - xx_3) / determinant * (xx_3 - xx_1) + (double)yy_1 / determinant * yy_1 * (xx_3 - xx_2) + (double)yy_2 * yy_2 / determinant * (xx_1 - xx_3) + (double)yy_3 * yy_3 / determinant * (xx_2 - xx_1)) / 2;
    double x = (xx_1 + xx_2 - ((xx_2 - xx_3) * (A_COFFIENT / (double)EE_1 - A_COFFIENT / (double)EE_2) - (yy_1 - yy_2) * (xx_2 - xx_3) * (double)(yy_1 + yy_2 - 2 * y)) / (xx_1 - xx_2) / (xx_2 - xx_3)) / 2;

    if (x < -POS_ERR_TOL || x > 254 + POS_ERR_TOL || y < -POS_ERR_TOL || y > 254 + POS_ERR_TOL) //????
        return 0;

    coordinate[0] = x;
    coordinate[1] = y;
    return 1;
}
void Sol_Car_Pos_INIT(){
    ///Sol_Car_Pos()初始化，赋值中间变量beacon_determinant
    ///进入第二回合时调用此函数进行赋值，或者把下面一行粘过去
    beacon_determinant = -beacon_Pos[0] * beacon_Pos[3] + beacon_Pos[2] * beacon_Pos[1]- beacon_Pos[4] * beacon_Pos[1] + beacon_Pos[0] * beacon_Pos[5] - beacon_Pos[2] * beacon_Pos[5] + beacon_Pos[4] * beacon_Pos[3];
}
void Sol_Car_Pos(double r_1,double r_2,double r_3){
    ///第二回合计算小车位置函数。计算出小车当前坐标，存储在car_Pos[2]数组中。入口参数：到信标1、2、3距离。
    ///返回值：无
    car_Pos[1]=(r_1*r_1*(beacon_Pos[4]-beacon_Pos[2])+r_3*r_3*(beacon_Pos[2]-beacon_Pos[0])+r_2*r_2*(beacon_Pos[0]-beacon_Pos[4]))/2/beacon_determinant+((beacon_Pos[2]-beacon_Pos[4])*beacon_Pos[1]*beacon_Pos[1]+(beacon_Pos[0]-beacon_Pos[2])*beacon_Pos[5]*beacon_Pos[5]+(beacon_Pos[4]-beacon_Pos[0])*beacon_Pos[3]*beacon_Pos[3])/2/beacon_determinant;
    car_Pos[0]=-(r_1*r_1*(beacon_Pos[5]-beacon_Pos[3])+r_3*r_3*(beacon_Pos[3]-beacon_Pos[1])+r_2*r_2*(beacon_Pos[1]-beacon_Pos[5]))/2/beacon_determinant-((beacon_Pos[3]-beacon_Pos[5])*beacon_Pos[0]*beacon_Pos[0]+(beacon_Pos[1]-beacon_Pos[3])*beacon_Pos[4]*beacon_Pos[4]+(beacon_Pos[5]-beacon_Pos[1])*beacon_Pos[2]*beacon_Pos[2])/2/beacon_determinant;
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

#ifdef USE_FULL_ASSERT
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
