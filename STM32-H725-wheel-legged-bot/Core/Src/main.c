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
#include "BMI088Middleware.h"  /* IMU middleware helper */
#include "BMI088driver.h"      /* IMU driver */

#include <string.h>
#include <stdio.h>
#include "bsp_fdcan.h"         /* FD-CAN helper */
#include <math.h>

#include "as5047p.h"           /* AS5047P encoder driver */
#include "odrive_can.h"        /* O-Drive CAN communication helper */
#include "joint_mit.h"         /* MIT CAN protocol control */
#include "joint_hw.h"          /* Joint-level hardware control */
#include "leg_kin.h"           /* Leg kinematics */
#include "crsf8.h"             /* CRSF RC receiver protocol */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Main loop: 1kHz update with 10Hz debug output */
#define PRINT_DIV               100   /* 1kHz / 100 = 10Hz print rate */
#define JOINT_COUNT             3     /* Three joints: hip_pitch, knee_pitch, hip_roll */
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
    /* Joint 0: Hip pitch */
    {
        .hspi=&hspi1, .cs_port=GPIOE, .cs_pin=GPIO_PIN_15,  /* Second encoder CS pin */
        .hcan=&hfdcan1, .node_id=2,                         /* O-Drive CAN node ID */
        .raw_min=3000, .raw_max=10000, .gear_ratio=1.0f,    /* Encoder range and gear ratio */
        .home_raw_abs=6100,                                 /* Home position */
        .kp=0.45f, .kd=0.025f, .tau_max=0.8f,               /* MIT control gains and max torque */
        .q_db=0.015f, .q_hold_band=0.020f, .qd_alpha=0.05f, .tau_rate_limit=10.0f,
        .torque_sign = -1,
        .ik_sign = -1
    },
    /* Joint 1: Knee pitch */
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
    /* Joint 2: Hip roll */
    {
        .hspi=&hspi1, .cs_port=GPIOB, .cs_pin=GPIO_PIN_11,
        .hcan=&hfdcan1, .node_id=1,
        .raw_min=2500, .raw_max=4000, .gear_ratio=1.0f,
        .home_raw_abs=2300,
        .kp=0.45f, .kd=0.025f, .tau_max=0.8f,
        .q_db=0.015f, .q_hold_band=0.020f, .qd_alpha=0.05f, .tau_rate_limit=10.0f,
		.torque_sign = -1,
		.ik_sign = 1
    }
};


/* Forward kinematics: leg link lengths */
static const LegGeom g_leg = { .l1 = 0.20f, .l2 = 0.20f };

/* RC receiver (CRSF protocol) */
static Crsf8 g_crsf;

/* IMU state: roll angle and rate */
typedef struct {
    float roll;        /* Roll angle (radians) */
    float roll_rate;   /* Roll rate from gyroscope (rad/s) */
    uint8_t ok;        /* Valid data flag */
} ImuState;

static ImuState g_imu = {0};
static ODriveCan g_wheel = {0};  /* Hub motor O-Drive (CAN node ID 3) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Utility: Clamp value to range [lo, hi] */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Utility: Map value from input range to output range */
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    float denom = (in_max - in_min);
    if (fabsf(denom) < 1e-6f) return out_min;

    float t = (x - in_min) / denom;
    t = clampf(t, 0.0f, 1.0f);
    return out_min + t * (out_max - out_min);
}

/* Inverse kinematics command for leg */
typedef struct {
    float x;      /* Forward/backward position (m), currently fixed at 0 */
    float z;      /* Leg height (m), controlled by RC CH3 */
    float roll;   /* Hip roll target (rad), controlled by RC CH1 */
} LegCmd;

/* Update leg command from RC receiver with low-pass filtering */
static void rc_update_cmd(LegCmd *cmd, const Crsf8 *crsf)
{
    /* RC CH3 -> leg height (m), range: -0.30 to -0.10 */
    uint16_t ch3 = crsf->ch8[2];
    static float z_f = -0.20f;
    float z_t = mapf((float)ch3, 180.0f, 1800.0f, -0.30f, -0.10f);
    z_f += 0.02f * (z_t - z_f);
    cmd->z = z_f;

    /* RC CH1 -> hip roll (±15 degrees with low-pass filter) */
    uint16_t ch1 = crsf->ch8[0];
    static float r_f = 0.0f;
    float r_t = mapf((float)ch1, 180.0f, 1800.0f,
                     -15.0f*(float)M_PI/180.0f,
                      15.0f*(float)M_PI/180.0f);
    r_f += 0.02f * (r_t - r_f);
    cmd->roll = r_f;

    cmd->x = 0.0f;  /* Forward/backward position fixed at 0 */
}
/* Solve inverse kinematics: Convert leg command to joint targets
 * Uses roll-led approach to maintain vertical height when rolling */
static int leg_solve_targets(const LegCmd *cmd, const LegGeom *g,
                             float *qr, float *qh, float *qk,
                             float *z_p_out)
{
    /* Protect against singular configuration when cosine is too small */
    float c = cosf(cmd->roll);
    if (fabsf(c) < 0.2f) c = (c > 0) ? 0.2f : -0.2f;

    /* Key: Adjust projected height to maintain vertical height as roll changes */
    float z_p = cmd->z / c;
    int ok = leg_ik_2d(cmd->x, z_p, g, qh, qk);
    if (ok != 0) return ok;

    *qr = cmd->roll;
    if (z_p_out) *z_p_out = z_p;
    return 0;
}
/* Apply smoothed joint targets with low-pass filter */
static void joints_apply_targets(float qr, float qh, float qk)
{
    const float a = 0.02f;  /* Low-pass filter coefficient */

    /* Apply sign correction based on IK configuration */
    float qh_m = (float)g_cfg[0].ik_sign * qh;
    float qk_m = (float)g_cfg[1].ik_sign * qk;
    float qr_m = (float)g_cfg[2].ik_sign * qr;

    /* Exponential smoothing of targets */
    g_joint[0].q_target += a * (qh_m - g_joint[0].q_target);
    g_joint[1].q_target += a * (qk_m - g_joint[1].q_target);
    g_joint[2].q_target += a * (qr_m - g_joint[2].q_target);
}

// ===== IMU state =====

/* Wrap angle to [-π, π] range */
static inline float wrap_pi(float a)
{
    while(a >  (float)M_PI) a -= 2.0f*(float)M_PI;
    while(a < -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}

/* 1kHz IMU update using complementary filter for roll angle estimation */
static int imu_update_1khz(ImuState *s, float dt)
{
    float gyro[3], accel[3], temp;

    /* Read raw IMU data from BMI088 sensor */
    BMI088_read(gyro, accel, &temp);

    /* Driver output units:
     * - gyro[] in rad/s (BMI088 gyro sensitivity ~0.001065 rad/s per LSB at ±2000dps)
     * - accel[] in g (gravity units) */
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];

    /* Gyroscope rate for X-axis rotation (roll rate) */
    float roll_rate = gyro[0];

    /* Estimate roll angle from gravity vector in accelerometer readings */
    float roll_acc = atan2f(ay, -az);

    /* Complementary filter: Fuse gyro integration and accelerometer estimate
     * Alpha ~0.99-0.999 for 1kHz sampling */
    const float alpha = 0.995f;
    float roll_pred = s->roll + roll_rate * dt;
    s->roll = wrap_pi(alpha * roll_pred + (1.0f - alpha) * roll_acc);

    s->roll_rate = roll_rate;
    s->ok = 1;
    return 0;
}

/* ===== Wheel Balance Control (PD) ===== */

/* Compute wheel torque command for balance using IMU feedback
 * Applies PD control: proportional to roll angle, derivative to roll rate */
static float wheel_tau_from_imu(const ImuState *imu)
{
    if (!imu->ok) return 0.0f;

    /* PD gains (tuning range: kp=0.2~1.5, kd=0.01~0.10) */
    const float kp = 0.6f;
    const float kd = 0.03f;

    /* PD control law: negative feedback stabilizes roll */
    float tau = -(kp * imu->roll + kd * imu->roll_rate);

    /* Limit output torque (typically 0.1~0.3 Nm for wheel motor) */
    const float tau_max = 0.25f;
    if (tau >  tau_max) tau =  tau_max;
    if (tau < -tau_max) tau = -tau_max;
    return tau;
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

  /* 1. Enable power supplies (GPIO control) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(50);

  /* 2. Initialize CAN bus and IMU */
  bsp_can_init();
  HAL_Delay(200);
  uint8_t imu_err = BMI088_init();
  UNUSED(imu_err);

  /* 3. Initialize all leg joints */
  for (int i = 0; i < JOINT_COUNT; i++) {
    joint_init(&g_joint[i], &g_cfg[i]);
  }

  /* 4. Enable torque control on all O-Drive motors */
  for (int i = 0; i < JOINT_COUNT; i++) {
    joint_odrive_enable_torque_mode(&g_joint[i]);
  }

  /* 5. Zero joint encoders to home positions */
  for (int i = 0; i < JOINT_COUNT; i++) {
      (void)joint_zero_to_home(&g_joint[i]);
  }

  /* 6. Initialize wheel O-Drive motor (CAN node 3) */
  g_wheel.hcan = &hfdcan1;
  g_wheel.node_id = 3;

  odrive_can_clear_errors(&g_wheel);
  HAL_Delay(10);

  /* Set O-Drive to torque control mode with passthrough */
  odrive_can_set_controller_mode(&g_wheel, 1, 1);
  HAL_Delay(10);

  /* Enable closed-loop control state */
  odrive_can_set_axis_state(&g_wheel, 8);
  HAL_Delay(10);

  /* 7. Start 1kHz main loop timer */
  HAL_TIM_Base_Start_IT(&htim2);

  /* 8. Initialize RC receiver and debug UART */
  crsf8_init(&g_crsf, &huart7);
  crsf8_start_rx_it(&g_crsf);
  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n[CRSF] Start\r\n", 16, 100);

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


      /* 1kHz: Update all leg joint states */
      for (int i = 0; i < JOINT_COUNT; i++) {
          joint_update_1khz(&g_joint[i]);
      }

      /* 1kHz: Update IMU and apply wheel balance control */
      imu_update_1khz(&g_imu, 0.001f);
      float tau = wheel_tau_from_imu(&g_imu);
      odrive_can_set_input_torque(&g_wheel, tau);

      /* 1kHz: Process RC input -> Compute trajectory -> Solve IK -> Update joint targets */
      static LegCmd cmd;
      static float z_p_dbg = 0.0f;
      static float qr=0, qh=0, qk=0;
      static int ok = 0;

      rc_update_cmd(&cmd, &g_crsf);
      ok = leg_solve_targets(&cmd, &g_leg, &qr, &qh, &qk, &z_p_dbg);
      if (ok == 0) joints_apply_targets(qr, qh, qk);

      /* 10Hz: Debug output to UART (1000ms / 100 = 10Hz) */
      if (++print_ctr >= PRINT_DIV) {
          print_ctr = 0;

          float tau_dbg = wheel_tau_from_imu(&g_imu);
          float roll_deg = g_imu.roll * 57.2957795f;   /* Convert radians to degrees */
          float rate_deg = g_imu.roll_rate * 57.2957795f;

          int n = snprintf(uart_buf, sizeof(uart_buf),
              "[IMU] roll=%.2f deg rate=%.2f deg/s | tau=%.3f Nm\r\n",
              (double)roll_deg, (double)rate_deg, (double)tau_dbg);

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

/* Timer interrupt callback: Triggers main control loop at 1kHz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    g_tick_1khz++;
    g_tick_flag = 1;  /* Signal main loop to execute */
  }
}

/* UART receive complete callback: Process incoming RC data byte-by-byte */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        crsf8_on_rx_byte(&g_crsf, g_crsf.rx_byte);
        /* Restart reception in interrupt mode for next byte */
        HAL_UART_Receive_IT(&huart7, &g_crsf.rx_byte, 1);
    }
}

/* UART error callback: Handle communication errors gracefully */
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
