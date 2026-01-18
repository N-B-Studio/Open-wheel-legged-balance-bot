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
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "BMI088driver.h"
#include "BMI088Middleware.h" // IMU reader helper
#include <string.h>
#include <stdio.h>
#include "bsp_fdcan.h"  // FD Can helper
#include <math.h>  

#include "as5047p.h"    // AS5047x Helper
#include "odrive_can.h" // Odrive Can communicate helper
#include "joint_mit.h" // mimic MIT CAN protical controll
#include "joint_hw.h" // overall joint level control
#include "leg_kin.h"
#include "crsf8.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== 1kHz loop -> 10Hz print =====
#define PRINT_DIV               100   // 1kHz/100=10Hz
#define JOINT_COUNT 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UNUSED(x) ((void)(x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_tick_1khz = 0;
volatile uint8_t  g_tick_flag = 0;
static char uart_buf[160];

static Joint g_joint[JOINT_COUNT];

static const JointConfig g_cfg[JOINT_COUNT] = {
    // J1: hip_pitch
    {
        .hspi=&hspi1, .cs_port=GPIOE, .cs_pin=GPIO_PIN_15, // Second Encoder CS pin
        .hcan=&hfdcan1, .node_id=2, 					   // O-drive can node Id
        .raw_min=3000, .raw_max=10000, .gear_ratio=1.0f,   // Second Encoder limit | gear_ratio = has second encoder? 1 : gear_ratio
        .home_raw_abs=6100,                                // Second Home position
        .kp=0.45f, .kd=0.025f, .tau_max=0.8f,              // MIT KP value
        .q_db=0.015f, .q_hold_band=0.020f, .qd_alpha=0.05f, .tau_rate_limit=10.0f, // Gear band, better the gear, smaller the number.
		.torque_sign = -1,
		.ik_sign = -1
    },
    // J2: knee_pitch
    {
        .hspi=&hspi1, .cs_port=GPIOB, .cs_pin=GPIO_PIN_10,
        .hcan=&hfdcan1, .node_id=6,
        .raw_min=5000, .raw_max=11700, .gear_ratio=1.0f,
        .home_raw_abs=11500,
        .kp=0.55f, .kd=0.03f, .tau_max=1.5f,
        .q_db=0.015f, .q_hold_band=0.020f, .qd_alpha=0.05f, .tau_rate_limit=10.0f,
		.torque_sign = -1,
		.ik_sign = -1
    },
    // J3: hip_roll
    {
        .hspi=&hspi1, .cs_port=GPIOB, .cs_pin=GPIO_PIN_11,
        .hcan=&hfdcan1, .node_id=1,
        .raw_min=2500, .raw_max=4000, .gear_ratio=1.0f,
        .home_raw_abs=2300,
        .kp=0.45f, .kd=0.025f, .tau_max=0.8f,
        .q_db=0.015f, .q_hold_band=0.020f, .qd_alpha=0.05f, .tau_rate_limit=10.0f,
		.torque_sign = -1,
		.ik_sign = -1
    }
};


// FK
static const LegGeom g_leg = { .l1 = 0.20f, .l2 = 0.20f };

// ELSR
static Crsf8 g_crsf;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// map int range -> float range
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    float t = (x - in_min) / (in_max - in_min);
    t = clampf(t, 0.0f, 1.0f);
    return out_min + t * (out_max - out_min);
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART10_UART_Init();
  MX_SPI1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  // 1) Power enable (Based on board requirements.)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(50);

  // 2) CAN init/start
  bsp_can_init();
  HAL_Delay(200);


  // init joints
  for (int i = 0; i < JOINT_COUNT; i++) {
	joint_init(&g_joint[i], &g_cfg[i]);
  }

  // enable odrive torque mode
  for (int i = 0; i < JOINT_COUNT; i++) {
	joint_odrive_enable_torque_mode(&g_joint[i]);
  }

  // zero to fixed home
  for (int i = 0; i < JOINT_COUNT; i++) {
      (void)joint_zero_to_home(&g_joint[i]);
  }

  
  // start 1kHz tick
  HAL_TIM_Base_Start_IT(&htim2);

  //const char *boot = "\r\n[H725] Joint1 MIT: AS5047P + MiniODrive torque @1kHz\r\n";
  //HAL_UART_Transmit(&huart1, (uint8_t*)boot, (uint16_t)strlen(boot), 100);


  crsf8_init(&g_crsf, &huart7);
  crsf8_start_rx_it(&g_crsf);
  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n[CRSF] start\r\n", 16, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_tick = 0;
  uint32_t print_ctr = 0;

  while (1)
  {
      if (!g_tick_flag) continue;
      g_tick_flag = 0;

      if (g_tick_1khz == last_tick) continue;
      last_tick = g_tick_1khz;


      // 1kHz update all joints
      for (int i = 0; i < JOINT_COUNT; i++) {
          joint_update_1khz(&g_joint[i]);
      }


      // =================== 1kHz: traj + IK + update targets ===================
      static float t = 0.0f;
      t += 0.006f;                 // ~1Hz 左右（可调）

      float x_cmd = 0.0f;

      // 竖直往返：[-0.30, -0.15]
      //float z_cmd = -0.225f + 0.075f * sinf(t);

      // ===== CRSF CH3 -> leg height z_cmd =====
      // CH3 expected: 180..1800
      // z_cmd: -0.30 .. -0.10
      uint16_t ch3 = g_crsf.ch8[2];

      // 可选：做个死区/滤波，避免抖动（先给你简单一阶滤波）
      static float z_cmd_f = -0.20f;
      float z_target = mapf((float)ch3, 180.0f, 1800.0f, -0.35f, -0.15f);

      // 低通：1kHz下 alpha 很小就够了
      const float z_alpha = 0.02f;
      z_cmd_f += z_alpha * (z_target - z_cmd_f);

      float z_cmd = z_cmd_f;

      // =============== CH3 end ==========================

      float qh = 0.0f, qk = 0.0f;
      int ok = leg_ik_2d(x_cmd, z_cmd, &g_leg, &qh, &qk);

      if (ok == 0) {
          float qh_m = (float)g_cfg[0].ik_sign * qh;
          float qk_m = (float)g_cfg[1].ik_sign * qk;

          const float alpha_1khz = 0.02f;   // 1kHz 下建议小一点（更顺）
          g_joint[0].q_target += alpha_1khz * (qh_m - g_joint[0].q_target);
          g_joint[1].q_target += alpha_1khz * (qk_m - g_joint[1].q_target);
          g_joint[2].q_target = 0.0f;
      }

      // =================== 10Hz: print only ===================
      if (++print_ctr >= PRINT_DIV) {
          print_ctr = 0;

          /*
          if (ok == 0) {
              int n = snprintf(uart_buf, sizeof(uart_buf),
                  "[IK] x=%.3f z=%.3f | qh=%.3f qk=%.3f | ik_sign=(%d,%d)\r\n"
                  "[J ] meas=(%.3f,%.3f) targ=(%.3f,%.3f) raw=(%u,%u)\r\n",
                  (double)x_cmd, (double)z_cmd,
                  (double)qh, (double)qk,
                  (int)g_cfg[0].ik_sign, (int)g_cfg[1].ik_sign,
                  (double)joint_get_q(&g_joint[0]), (double)joint_get_q(&g_joint[1]),
                  (double)g_joint[0].q_target, (double)g_joint[1].q_target,
                  (unsigned)g_joint[0].last_raw, (unsigned)g_joint[1].last_raw
              );
              HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, 100);
          } else {
              HAL_UART_Transmit(&huart1, (uint8_t*)"[IK] unreachable\r\n", 18, 100);
          }
          */
          uint32_t age = crsf8_age_ms(&g_crsf);
          uint16_t ch3 = g_crsf.ch8[2];
          int n = snprintf(uart_buf, sizeof(uart_buf),
            "[CRSF] age=%lums ok=%lu bad=%lu | ch3=%u z=%.3f | ch=%u %u %u %u %u %u %u %u\r\n",
            (unsigned long)age,
            (unsigned long)g_crsf.ok,
            (unsigned long)g_crsf.bad,
            (unsigned)ch3,
            (double)z_cmd,
            (unsigned)g_crsf.ch8[0], (unsigned)g_crsf.ch8[1], (unsigned)g_crsf.ch8[2], (unsigned)g_crsf.ch8[3],
            (unsigned)g_crsf.ch8[4], (unsigned)g_crsf.ch8[5], (unsigned)g_crsf.ch8[6], (unsigned)g_crsf.ch8[7]
          );

          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, (uint16_t)n, 100);


      }
  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
	g_tick_1khz++;
	g_tick_flag = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        crsf8_on_rx_byte(&g_crsf, g_crsf.rx_byte);
        HAL_UART_Receive_IT(&huart7, &g_crsf.rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        crsf8_on_uart_error(&g_crsf);
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
