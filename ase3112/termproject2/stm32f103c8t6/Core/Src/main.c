#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 전역 변수 */
MPU6050_t MPU6050;

uint8_t rxBuffer[1];
uint8_t messageBuffer[512];
uint16_t messageIndex = 0;

double roll = 0.0, pitch = 0.0, yaw = 0.0;
double baseDuty1 = 0, baseDuty2 = 0, baseDuty3 = 0, baseDuty4 = 0;

double previousGz = 0.0;
double alpha = 0.5;  // 저역통과필터 계수
uint32_t tick_prev = 0;

int enabled = 0;               // 모터 제어 활성 여부 (초기 정지 상태)
extern int enabled;            // usart.c에서 접근 가능하게

/* 함수 원형 */
void SystemClock_Config(void);
void setMotorDuty(uint8_t channel, double dutyPercent);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MPU6050_Init(&hi2c2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_UART_Receive_IT(&huart2, rxBuffer, 1);

  char welcomeMsg[] = "UART2 Ready. System Start!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)welcomeMsg, strlen(welcomeMsg), HAL_MAX_DELAY);

  while (1)
  {
    uint32_t tick_now = HAL_GetTick();
    double dt = (tick_now - tick_prev) * 0.001; // seconds
    if (dt < 0.001) dt = 0.001;
    tick_prev = tick_now;

    MPU6050_Read_All(&hi2c2, &MPU6050);

    pitch = MPU6050.KalmanAngleY + 90 ;

    double currentGz = MPU6050.Gz;
    double filteredGz = alpha * previousGz + (1.0 - alpha) * currentGz;
    roll += filteredGz * dt;
    previousGz = filteredGz;

    char modeMsg[64];

    if (enabled)  // "start" 명령이 수신된 경우에만 동작
    {
      if (pitch < -40) {
        baseDuty1 = 0; baseDuty2 = 80; baseDuty3 = 0; baseDuty4 = 80;
        strcpy(modeMsg, "Mode 1: Pitch Down");
      } else if (roll < -40) {
        baseDuty1 = 80; baseDuty2 = 0; baseDuty3 = 0; baseDuty4 = 80;
        strcpy(modeMsg, "Mode 2: Roll Left (CCW)");
      } else if (roll > 40) {
        baseDuty1 = 0; baseDuty2 = 80; baseDuty3 = 80; baseDuty4 = 0;
        strcpy(modeMsg, "Mode 3: Roll Right (CW)");
      } else if (pitch > 40) {
        baseDuty1 = 80; baseDuty2 = 0; baseDuty3 = 80; baseDuty4 = 0;
        strcpy(modeMsg, "Mode 4: Pitch Up");
      } else {
        baseDuty1 = baseDuty2 = baseDuty3 = baseDuty4 = 20;
        strcpy(modeMsg, "Stable");
      }
    }
    else  //"stop" 상태일 때는 모터 정지
    {
      baseDuty1 = baseDuty2 = baseDuty3 = baseDuty4 = 0;
      strcpy(modeMsg, "Motors OFF (Waiting)");
    }

    setMotorDuty(0, baseDuty1);
    setMotorDuty(1, baseDuty2);
    setMotorDuty(2, baseDuty3);
    setMotorDuty(3, baseDuty4);

    char debugMsg[128];
    snprintf(debugMsg, sizeof(debugMsg),
         "R:%d, P:%d %s\r\n",
         (int)roll, (int)pitch, modeMsg);


    HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), HAL_MAX_DELAY);
    HAL_Delay(100);
  }
}

void setMotorDuty(uint8_t channel, double dutyPercent)
{
  if (dutyPercent < 0) dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;

  uint16_t periodTicks = TIM1->ARR + 1;
  uint16_t ccr = (uint16_t)((dutyPercent / 100.0) * periodTicks);

  switch (channel)
  {
    case 0: TIM1->CCR1 = ccr; break;
    case 1: TIM1->CCR2 = ccr; break;
    case 2: TIM1->CCR3 = ccr; break;
    case 3: TIM1->CCR4 = ccr; break;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // Optional
}
#endif
