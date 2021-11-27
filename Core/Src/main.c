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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//////////////////////////////////////基本信息///////////////////////////////////////////
//定时器周期为50ms

volatile float distance1 = 0.0, distance2 = 0.0, distance3 = 0.0, distance4 = 0.0;
extern uint8_t jy62Receive[JY62_MESSAGE_LENGTH];
extern uint8_t jy62Message[JY62_MESSAGE_LENGTH];

float angle_obj, dis_obj;         //未使用的全局变量??
float obj_x, obj_y, car_x, car_y; //未使用的全局变量??

//////////////////////////////////////Sol_Car_Pos函数变量定义///////////////////////////////////////////
int beacon_Pos[6];               //三个信标坐标，依次存放x_1、y_1、x_2、y_2、x_3、y_3
int car_Pos[2];                  //小车当前坐标
double beacon_determinant = 0.0; //计算中间变量
//////////////////////////////////////Sol_Car_Pos函数定义结束///////////////////////////////////////////
//////////////////////////////////////求解金矿位置中间变量容器///////////////////////////////////////////
//求解金矿位置�??要用到当前时刻�?�上�??时刻、上上个时刻的小车位置与金矿强度。此容器用于存储这些变量
struct Sol_Mine_Pos_Temp
{
    uint16_t x;   //小车x坐标
    uint16_t y;   //小车y坐标
    uint32_t E_1; //场强1
    uint32_t E_2; //场强2
} Prev_Pos[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
short Prev_Pos_head = 0;
////////////////////////////////////求解金矿位置中间变量容器结束/////////////////////////////////////////

struct A //资源坐标
{
    int x[2];         //(x[0],y[0])是第�??个资源的坐标
    int y[2];         //(x[1],y[1])是第二个资源的坐�??
    int iscalculated; //是否被计算出�?? 0-false
    int priority;     //�??近资源的下标
} resource_location = {{-1, -1}, {-1, -1}, 0, -1};
int score;                //得分�??
int count_pos = 0;        //坐标计数器，匹配上位机频率
int count_beacon = 0;     //信标计数器，确保停留一秒
int count_storehouse = 0; //仓库计时器，确保停留两秒
uint16_t now;             //记录当前时间

int goto_state = 0;
float dis_cur, dis_pre;
//仓库坐标
int rep_not_ordered[8][2] = {{15, 15},
                             {127, 15},
                             {239, 127},
                             {239, 239},
                             {127, 239},
                             {15, 239},
                             {15, 127}};

int State = -1;
//状�?�变量对应表
// -1-比赛未开始或已结�?
// 0-初始/随意游走解资源坐�?
// 1-已获得资源坐标，去第�?个资�?
// 2-已捡起第�?个资源，去第二个
// 3-资源全部捡到，去放第�?个信�?(beacon)
// 4-去放第二个信�?
// 5-去放第三个信�?
// 6-去仓�?

int destination[2]; //放到全局
int prev_type;      //全局 记录上一次抵达仓库的种类

#define forward_speed 1500
#define rotate_speed 3000
#define angle_err 2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//////////////////////////////////////Solve_Mine_Pos函数宏定�???///////////////////////////////////////////
#define A_COFFIENT 1000000000.0 //金矿强度系数A
#define POS_ERR_TOL 4           //金矿位置计算结果偏离场地的最大误�???
//////////////////////////////////////Solve_Mine_Pos宏定义结�???///////////////////////////////////////////
////////////////////////////////////////atan2LUTif函数宏定�???/////////////////////////////////////////////
#define M_PI_2 1.5707963
#define M_PI 3.141592654
#define M_PI_4_P_0273 1.05839816339744830962 //M_PI/4 + 0.273
const double ATAN_LUT[256] = {0.0000000000, 0.0039215485, 0.0078429764, 0.0117641631, 0.0156849881, 0.0196053309,
                              0.0235250710, 0.0274440882, 0.0313622624, 0.0352794736, 0.0391956019, 0.0431105278,
                              0.0470241318, 0.0509362949, 0.0548468980, 0.0587558227, 0.0626629506, 0.0665681638,
                              0.0704713446, 0.0743723758, 0.0782711405, 0.0821675224, 0.0860614053, 0.0899526737,
                              0.0938412126, 0.0977269074, 0.1016096438, 0.1054893085, 0.1093657884, 0.1132389710,
                              0.1171087446, 0.1209749978, 0.1248376255, 0.1286965013, 0.1325515323, 0.1364026044,
                              0.1402496096, 0.1440924408, 0.1479309912, 0.1517651553, 0.1555948280, 0.1594199049,
                              0.1632402828, 0.1670558588, 0.1708665312, 0.1746721990, 0.1784727620, 0.1822681208,
                              0.1860581771, 0.1898428334, 0.1936219929, 0.1973955598, 0.2011634395, 0.2049255380,
                              0.2086817623, 0.2124320205, 0.2161762215, 0.2199142752, 0.2236460927, 0.2273715857,
                              0.2310906672, 0.2348032511, 0.2385092525, 0.2422085871, 0.2459011721, 0.2495869254,
                              0.2532657662, 0.2569376146, 0.2606023917, 0.2642600199, 0.2679104224, 0.2715535237,
                              0.2751892491, 0.2788175253, 0.2824382800, 0.2860514417, 0.2896569404, 0.2932547070,
                              0.2968446734, 0.3004267728, 0.3040009393, 0.3075671084, 0.3111252164, 0.3146752558,
                              0.3182170002, 0.3217505544, 0.3252758042, 0.3287926915, 0.3323011594, 0.3358011520,
                              0.3392926145, 0.3427754932, 0.3462497357, 0.3497152904, 0.3531721069, 0.3566201360,
                              0.3600593294, 0.3634896400, 0.3669110217, 0.3703234297, 0.3737268255, 0.3771211497,
                              0.3805063771, 0.3838824615, 0.3872493632, 0.3906070437, 0.3939554653, 0.3972945915,
                              0.4006243869, 0.4039448169, 0.4072558481, 0.4105574480, 0.4138495853, 0.4171322295,
                              0.4204053512, 0.4236689219, 0.4269229141, 0.4301673014, 0.4334020581, 0.4366271598,
                              0.4398425828, 0.4430483044, 0.4462443029, 0.4494305575, 0.4526070482, 0.4557737560,
                              0.4589306629, 0.4620777516, 0.4652150058, 0.4683424102, 0.4714599501, 0.4745676117,
                              0.4776653824, 0.4807532499, 0.4838312032, 0.4868992318, 0.4899573263, 0.4930054778,
                              0.4960436784, 0.4990719209, 0.5020901990, 0.5050985071, 0.5080968402, 0.5110851942,
                              0.5140635659, 0.5170319525, 0.5199903521, 0.5229387636, 0.5258771863, 0.5288056206,
                              0.5317240673, 0.5346325278, 0.5375310045, 0.5404195003, 0.5432980185, 0.5461665634,
                              0.5490251398, 0.5518737530, 0.5547124091, 0.5575411147, 0.5603598769, 0.5631687036,
                              0.5659676030, 0.5687565842, 0.5715356566, 0.5743048302, 0.5770641155, 0.5798135236,
                              0.5825530662, 0.5852827553, 0.5880026035, 0.5907126240, 0.5934128303, 0.5961032364,
                              0.5987838570, 0.6014547069, 0.6041158015, 0.6067671569, 0.6094087892, 0.6120407151,
                              0.6146629519, 0.6172755171, 0.6198784285, 0.6224717045, 0.6250553640, 0.6276294258,
                              0.6301939095, 0.6327488350, 0.6352942223, 0.6378300921, 0.6403564651, 0.6428733625,
                              0.6453808058, 0.6478788169, 0.6503674179, 0.6528466311, 0.6553164793, 0.6577769856,
                              0.6602281731, 0.6626700655, 0.6651026865, 0.6675260602, 0.6699402110, 0.6723451634,
                              0.6747409422, 0.6771275725, 0.6795050796, 0.6818734889, 0.6842328261, 0.6865831172,
                              0.6889243882, 0.6912566655, 0.6935799756, 0.6958943450, 0.6981998008, 0.7004963699,
                              0.7027840796, 0.7050629571, 0.7073330300, 0.7095943260, 0.7118468729, 0.7140906986,
                              0.7163258312, 0.7185522990, 0.7207701302, 0.7229793534, 0.7251799971, 0.7273720900,
                              0.7295556609, 0.7317307387, 0.7338973524, 0.7360555311, 0.7382053040, 0.7403467003,
                              0.7424797493, 0.7446044805, 0.7467209234, 0.7488291075, 0.7509290624, 0.7530208178,
                              0.7551044035, 0.7571798492, 0.7592471847, 0.7613064400, 0.7633576449, 0.7654008294,
                              0.7674360235, 0.7694632573, 0.7714825607, 0.7734939638, 0.7754974968, 0.7774931897,
                              0.7794810727, 0.7814611759, 0.7834335294, 0.7853981634};
////////////////////////////////////////atan2LUTif宏定义结�???/////////////////////////////////////////////

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
        if (State < 10)
        {
            count_pos++; //进两次定时器更新一次小车xy坐标
            if (count_pos == 2)
            {
                car_Pos[0] = getCarPosX();
                car_Pos[1] = getCarPosY(); ///更新小车xy坐标
                count_pos = 0;
            }
        }
        else
        {
            uint16_t r_0, r_1, r_2; //小车到信标的距离 放到全局
            r_0 = getDistanceOfMyBeacon(0);
            r_1 = getDistanceOfMyBeacon(1);
            r_2 = getDistanceOfMyBeacon(2);
            Sol_Car_Pos(r_0, r_1, r_2); ///更新小车xy坐标
        }

        switch (State)
        {
        case (0): //初始状�??
        {
            ////////////////////////这一部分应当仔细�??�??////////////////////////////////
            ////////////////////////这一部分应当仔细�??�??////////////////////////////////
            ////////////////////////这一部分应当仔细�??�??////////////////////////////////
            //步骤1：随意走�??
            rotate_clockwise_plus_forward(rotate_speed); ///初始时转圈，确定资源坐标
            //步骤2：解算两个资源坐�?? =====待实�??=====
            // if (resource_location.iscalculated == 1) //如果测算出坐�??
            // {
            //     State = 1; //进入下一状�??
            //     //步骤3：确定离车最近的�??�??
            //     resource_location.priority = (resource_location.x[0]*resource_location.x[0]+resource_location.y[0]*resource_location.y[0])<(resource_location.x[1]*resource_location.x[1]+resource_location.y[1]*resource_location.y[1])?0:1;   //确定距离更近�??
            // }
            // break;
            if (Solve_Mine_Pos(Prev_Pos[0].x, Prev_Pos[0].y, Prev_Pos[0].E_1, Prev_Pos[1].x, Prev_Pos[1].y, Prev_Pos[1].E_1, Prev_Pos[2].x, Prev_Pos[2].y, Prev_Pos[2].E_1, resource_location.x, resource_location.y) && Solve_Mine_Pos(Prev_Pos[0].x, Prev_Pos[0].y, Prev_Pos[0].E_2, Prev_Pos[1].x, Prev_Pos[1].y, Prev_Pos[1].E_2, Prev_Pos[2].x, Prev_Pos[2].y, Prev_Pos[2].E_2, resource_location.x + 1, resource_location.y + 1))
            {
                //利用Solve_Mine_Pos函数返回值直接判断计算是否成�??
                State = 1; //进入下一状�??
                //步骤3：确定离车最近的�??�??
                resource_location.priority = (resource_location.x[0] * resource_location.x[0] + resource_location.y[0] * resource_location.y[0]) < (resource_location.x[1] * resource_location.x[1] + resource_location.y[1] * resource_location.y[1]) ? 0 : 1; //确定距离更近�??
                break;
            }
            else
            {
                //case计算不成功，�??要更新位置与场强信息，重新计算�??
                Prev_Pos[Prev_Pos_head].x = getCarPosX();          //利用getCarPosX函数获取小车x坐标进行更新
                Prev_Pos[Prev_Pos_head].y = getCarPosY();          //同上
                Prev_Pos[Prev_Pos_head].E_1 = getMineIntensity(0); //利用getMineIntensity函数更新场强
                Prev_Pos[Prev_Pos_head].E_2 = getMineIntensity(1); //同上
                Prev_Pos_head = (Prev_Pos_head + 1) % 3;           //头指针循环地后移
                break;
            }
        }
        case (1):
        {
            //去最近的资源
            if (goto_state == 0)
                Goto(resource_location.x[resource_location.priority], resource_location.y[resource_location.priority]);
            dis_cur = (resource_location.x[resource_location.priority] - car_Pos[0]) * (resource_location.x[resource_location.priority] - car_Pos[0]) + (resource_location.y[resource_location.priority] - car_Pos[1]) * (resource_location.y[resource_location.priority] - car_Pos[1]);
            if (getCarMineSumNum() == 1)
            {
                State = 2; //进入下一状�??
                goto_state = 0;
                // score = getCarScore(); //更新当前分数
            }
            if (goto_state)
            {
                if (dis_cur > dis_pre + 300)
                    goto_state = 0;
            }
            dis_pre = dis_cur;
            break;
        }
        case (2):
        {
            //去第二个资源
            if (goto_state == 0)
                Goto(resource_location.x[!resource_location.priority], resource_location.y[!resource_location.priority]);
            dis_cur = (resource_location.x[!resource_location.priority] - car_Pos[0]) * (resource_location.x[!resource_location.priority] - car_Pos[0]) + (resource_location.y[!resource_location.priority] - car_Pos[1]) * (resource_location.y[!resource_location.priority] - car_Pos[1]);
            if (getCarMineSumNum() == 2)
            {
                State = 3; //进入下一状�??
                goto_state = 0;
                // score = getCarScore(); //更新当前分数
            }
            if (goto_state)
            {
                if (dis_cur > dis_pre + 300)
                    goto_state = 0;
            }
            dis_pre = dis_cur;
            break;
        }
        case (3):
        {
            //放置第一个信�? (127,60）中偏上
            if ((car_Pos[0] - 127) * (car_Pos[0] - 127) + (car_Pos[1] - 30) * (car_Pos[1] - 30) < 10)
            {
                brake(); //刹车
                count_beacon++;
                if (count_beacon <= 20)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                }
                else
                {
                    State = 4;
                    count_beacon = 0;
                    beacon_Pos[0] = getCarPosX();
                    beacon_Pos[1] = getCarPosY(); //给beacon_Pos赋�?�，用于第二回合精确计算坐标
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                    goto_state = 0;
                }
            }
            else
            {
                if (goto_state == 0)
                    Goto(127, 30);
            }

            break;
        }
        case (4):
        {
            //放置第二个信�?? (127,127)正中�??
            if ((car_Pos[0] - 127) * (car_Pos[0] - 127) + (car_Pos[1] - 127) * (car_Pos[1] - 127) < 15)
            {
                brake(); //刹车
                count_beacon++;
                if (count_beacon <= 20)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                }
                else
                {
                    State = 5;
                    count_beacon = 0;
                    beacon_Pos[2] = getCarPosX();
                    beacon_Pos[3] = getCarPosY(); //给beacon_Pos赋�?�，用于第二回合精确计算坐标
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                    goto_state = 0;
                }
            }
            else
            {
                if (goto_state == 0)
                    Goto(127, 60);
            }
            break;
        }
        case (5):
        {
            //放置第三个信�?? (60,127)中偏�??
            if ((car_Pos[0] - 60) * (car_Pos[0] - 60) + (car_Pos[1] - 127) * (car_Pos[1] - 127) < 15)
            {
                brake(); //刹车
                count_beacon++;
                if (count_beacon <= 20)
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                }
                else
                {
                    State = 6;
                    count_beacon = 0;
                    beacon_Pos[4] = getCarPosX();
                    beacon_Pos[5] = getCarPosY(); //给beacon_Pos赋�?�，用于第二回合精确计算坐标
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                    goto_state = 0;
                }
            }
            else
            {
                if (goto_state == 0)
                    Goto(127, 30);
            }
            break;
        }
        case (6):
        {
            //去最近仓�?7号，不判断是否到�??
            //还是判断一下吧
            if ((car_Pos[0] - 15) * (car_Pos[0] - 15) + (car_Pos[1] - 127) * (car_Pos[1] - 127) < 15)
            {
                brake();    //刹车
                State = -1; //第一回合结束
            }
            else
                Goto(15, 127);
            break;
        }
        case (10):
        {
            score = getCarScore();
            //求解资源坐标
            if (Solve_Mine_Pos(Prev_Pos[0].x, Prev_Pos[0].y, Prev_Pos[0].E_1, Prev_Pos[1].x, Prev_Pos[1].y, Prev_Pos[1].E_1, Prev_Pos[2].x, Prev_Pos[2].y, Prev_Pos[2].E_1, resource_location.x, resource_location.y) && Solve_Mine_Pos(Prev_Pos[0].x, Prev_Pos[0].y, Prev_Pos[0].E_2, Prev_Pos[1].x, Prev_Pos[1].y, Prev_Pos[1].E_2, Prev_Pos[2].x, Prev_Pos[2].y, Prev_Pos[2].E_2, resource_location.x + 1, resource_location.y + 1))
            {
                //利用Solve_Mine_Pos函数返回值直接判断计算是否成�?
                State = 11; //进入行进模式
                //确定离车最近的资源
                resource_location.priority = (resource_location.x[0] * resource_location.x[0] + resource_location.y[0] * resource_location.y[0]) < (resource_location.x[1] * resource_location.x[1] + resource_location.y[1] * resource_location.y[1]) ? 0 : 1; //确定距离更近�?
            }
            else
            {
                //case计算不成功，�?要更新位置与场强信息，重新计算�??
                Prev_Pos[Prev_Pos_head].x = getCarPosX();          //利用getCarPosX函数获取小车x坐标进行更新
                Prev_Pos[Prev_Pos_head].y = getCarPosY();          //同上
                Prev_Pos[Prev_Pos_head].E_1 = getMineIntensity(0); //利用getMineIntensity函数更新场强
                Prev_Pos[Prev_Pos_head].E_1 = getMineIntensity(1); //同上
                Prev_Pos_head = (Prev_Pos_head + 1) % 3;           //头指针循环地后移
            }
            break;
        }
        case (11):
        {
            Goto(resource_location.x[!resource_location.priority], resource_location.y[!resource_location.priority]);
            if (getCarScore() > score)
            {
                State = 10;                   //返回计算状态
                if (getCarMineSumNum() == 10) //满载
                    State = 12;               //而归
            }
            break;
        }
        case (12):
        {
            //计算小车到8个仓库的距离
            Get_Rep_opt(car_Pos[0], car_Pos[1]);
            State = 13;
        }
        case (13):
        {
            Goto(destination[0], destination[1]); //前往仓库
            if (getCarScore() > score)            //运送成功
            {
                State = 12;                  //返回计算状态
                if (getCarMineSumNum() == 0) //全部卸载
                    State = 10;
            }
            break;
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
        // rotate_clockwise_plus_forward(rotate_speed);
        // u1_printf(getGameState());
        // u1_printf("\n");
        // rotate_clockwise(rotate_speed);
        // Goto(20, 20);
        // HAL_Delay(100);
        if (getGameState() == 3)
        {
            State = -1;
        }
        if ((getGameState() == 1) && (getCarTask() == 0) && (State < 0)) //上半�?
        {
            State = 0;
        }
        if ((getGameState() == 1) && (getCarTask() == 1)) //下半�?
        {
            if (State < 10)
            {
                State = 10;
                Sol_Car_Pos_INIT();
            }
            if ((getGameTime() > 1000) && (State < 12)) //�?后时�?
                State = 12;
        }
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

void rotate_clockwise_plus_forward(int pwm)
{
    //change rotate direction of wheel
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //右边
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //左边
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //give pwm output
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 800);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 800);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 2000);
}
void brake()
{
    //change rotate direction of wheel
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    //give pwm output
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
}
double fabs(double x)
{
    //双精度浮点数求绝对�?�函�??
    return (x > 0) ? x : (-x);
}
double atan2LUTif(double y, double x)
{
    //反正切函数实�??.误差不大�??0.5°.入口参数：两点间y、x坐标的差�??
    double absx, absy, val;
    if (x == 0 && y == 0)
    {
        return 0;
    }
    absy = fabs(y);
    absx = fabs(x);
    if (absy - absx == absy)
    {
        return y < 0 ? -M_PI_2 : M_PI_2;
    }
    if (absx - absy == absx)
    {
        val = 0.0;
    }
    else
    {
        if (y > 0)
        {
            if (absx > absy)
                val = ATAN_LUT[(int)(255 * absy / absx)]; //1st octant
            else
                val = M_PI_2 - ATAN_LUT[(int)(255 * absx / absy)]; //2nd octant
            val = x < 0 ? (M_PI - val) : val;                      //3-4th octants from 2-1
        }
        else
        {
            if (absx > absy)
                val = -ATAN_LUT[(int)(255 * absy / absx)]; //8th octant
            else
                val = -M_PI_2 + ATAN_LUT[(int)(255 * absx / absy)]; //7th octant
            val = x < 0 ? -M_PI - val : val;                        //5-6th octants from 8-7
        }
    }
    val = val * 180 / M_PI;
    return (val < 0) ? (val + 360) : val;
}
int Solve_Mine_Pos(uint16_t xx_1, uint16_t yy_1, uint32_t EE_1, uint16_t xx_2, uint16_t yy_2, uint32_t EE_2, uint16_t xx_3, uint16_t yy_3, uint32_t EE_3, int *coordinate_x, int *coordinate_y)
{
    /*
    入口参数：xx_i,yy_i,EE_i,(i=1,2,3)为三个不同点的坐标与场强。double *coordinate是用于存放计算结果（金矿坐标）的容器
    此函数根据三组坐�??&场强数据进行解算，将得到的结果存储在coordinate数组�??.在较坏的情况下，误差�??3cm以内.
    返回值为0or1。case 1：计算无明显异常;case 0：三个点数据量不够，或是计算结果偏出场地以外。需重新计算.
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

    *coordinate_x = (int)x;
    *coordinate_y = (int)y;
    return 1;
}
void Sol_Car_Pos_INIT()
{
    ///Sol_Car_Pos()初始化，赋�?�中间变量beacon_determinant
    ///进入第二回合时调用此函数进行赋�?�，或�?�把下面�??行粘过去
    beacon_determinant = -beacon_Pos[0] * beacon_Pos[3] + beacon_Pos[2] * beacon_Pos[1] - beacon_Pos[4] * beacon_Pos[1] + beacon_Pos[0] * beacon_Pos[5] - beacon_Pos[2] * beacon_Pos[5] + beacon_Pos[4] * beacon_Pos[3];
}
void Sol_Car_Pos(double r_1, double r_2, double r_3)
{
    ///第二回合计算小车位置函数。计算出小车当前坐标，存储在car_Pos[2]数组中�?�入口参数：到信�??1�??2�??3距离�??
    ///返回值：�??
    car_Pos[1] = (r_1 * r_1 * (beacon_Pos[4] - beacon_Pos[2]) + r_3 * r_3 * (beacon_Pos[2] - beacon_Pos[0]) + r_2 * r_2 * (beacon_Pos[0] - beacon_Pos[4])) / 2 / beacon_determinant + ((beacon_Pos[2] - beacon_Pos[4]) * beacon_Pos[1] * beacon_Pos[1] + (beacon_Pos[0] - beacon_Pos[2]) * beacon_Pos[5] * beacon_Pos[5] + (beacon_Pos[4] - beacon_Pos[0]) * beacon_Pos[3] * beacon_Pos[3]) / 2 / beacon_determinant;
    car_Pos[0] = -(r_1 * r_1 * (beacon_Pos[5] - beacon_Pos[3]) + r_3 * r_3 * (beacon_Pos[3] - beacon_Pos[1]) + r_2 * r_2 * (beacon_Pos[1] - beacon_Pos[5])) / 2 / beacon_determinant - ((beacon_Pos[3] - beacon_Pos[5]) * beacon_Pos[0] * beacon_Pos[0] + (beacon_Pos[1] - beacon_Pos[3]) * beacon_Pos[4] * beacon_Pos[4] + (beacon_Pos[5] - beacon_Pos[1]) * beacon_Pos[2] * beacon_Pos[2]) / 2 / beacon_determinant;
}
void Goto(int x, int y)
{
    float angle_obj;
    angle_obj = 360 - atan2LUTif(y - car_Pos[1], x - car_Pos[0]); ///计算到目标点连线的夹角�?�car_Pos为小车坐�??

    float angle_car = GetYaw();
    if (angle_obj >= angle_err && angle_obj <= 360.0 - angle_err)
    {
        if (angle_car >= angle_obj - angle_err && angle_car <= angle_obj + angle_err)
        {
            forward(forward_speed);
            goto_state = 1;
            return;
        }
        else
        {
            if (angle_obj <= 180)
            {
                if (angle_car >= angle_obj && angle_car <= angle_obj + 180)
                {
                    rotate_clockwise(rotate_speed);
                }
                else
                    rotate_counterclockwise(rotate_speed);
            }
            else
            {
                if (angle_car <= angle_obj && angle_car >= angle_obj - 180)
                {
                    rotate_counterclockwise(rotate_speed);
                }
                else
                    rotate_clockwise(rotate_speed);
            }
        }
    }
    else
    {
        if (angle_obj < angle_err)
        {
            if ((angle_car >= angle_obj - angle_err + 360.0 || angle_car <= angle_obj + angle_err))
            {
                forward(forward_speed);
                goto_state = 1;
                return;
            }
            else
            {
                if (angle_car >= angle_obj && angle_car <= angle_obj + 180)
                {
                    rotate_clockwise(rotate_speed);
                }
                else
                    rotate_counterclockwise(rotate_speed);
            }
        }
        else if (angle_obj > 360 - angle_err)
        {
            if ((angle_car <= angle_obj + angle_err - 360.0 || angle_car >= angle_obj - angle_err))
            {
                forward(forward_speed);
                goto_state = 1;
                return;
            }
            else
            {
                if (angle_car <= angle_obj && angle_car >= angle_obj - 180)
                {
                    rotate_counterclockwise(rotate_speed);
                }
                else
                    rotate_clockwise(rotate_speed);
            }
        }
    }
}

//寻找�??近的仓库坐标
// Parameters: 小车坐标
// Return: �??近的仓库坐标的数组指�??
void Get_Rep_opt(int x, int y)
{
    int min;
    int min_rep;
    int i = 0;
    min = (x - 15) * (x - 15) + (y - 15) * (y - 15);
    min_rep = 0;
    for (i = 0; i < 8; ++i)
    {
        if (getParkDotMineType(i) == prev_type) //同类型已经去过了
            continue;
        if (((rep[i][0] - x) * (rep[i][0] - x) + (rep[i][1] - y) * (rep[i][1] - y)) < min)
        {
            min_rep = i;
            min = (rep[i][0] - x) * (rep[i][0] - x) + (rep[i][1] - y) * (rep[i][1] - y);
        }
    }
    for (i = 0; i < 3; ++i)
    {
        if (getMyBeaconMineType(i) == prev_type) //同类型已经去过了
            continue;
        if (((beacon_Pos[2 * i] - x) * (beacon_Pos[2 * i] - x) + (beacon_Pos[2 * i + 1] - y) * (beacon_Pos[2 * i + 1] - y)) < min)
        {
            min_rep = 8 + i; //8到10
            min = (beacon_Pos[2 * i] - x) * (beacon_Pos[2 * i] - x) + (beacon_Pos[2 * i + 1] - y) * (beacon_Pos[2 * i + 1] - y);
        }
    }
    if (min_rep < 8)
    {
        destination[0] = rep[min_rep][0];
        destination[1] = rep[min_rep][1];
        //return rep[min_rep];
    }
    else
    {
        destination[0] = beacon_Pos[2 * (min_rep - 8)];
        destination[1] = beacon_Pos[2 * (min_rep - 8) + 1];
        //return {beacon_Pos[2 * (min_rep - 8)], beacon_Pos[2 * (min_rep - 8) + 1]};}
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
