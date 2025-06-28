/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* 외부 전역 변수 */
extern uint8_t rxBuffer[1];
extern uint8_t messageBuffer[512];
extern uint16_t messageIndex;
extern double baseDuty1, baseDuty2, baseDuty3, baseDuty4;
extern int enabled;
extern double roll;     // reset 명령어 처리를 위한 외부 변수
extern double pitch;

UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t receivedChar = rxBuffer[0];

    // 필터: 유효한 ASCII 문자만 처리
    if ((receivedChar >= 32 && receivedChar <= 126) || receivedChar == '\r' || receivedChar == '\n')
    {
      if (receivedChar == '\r' || receivedChar == '\n')
      {
        if (messageIndex > 0)
        {
          messageBuffer[messageIndex] = '\0';

          // 줄바꿈 제거
          size_t len = strlen((char*)messageBuffer);
          while (len > 0 && (messageBuffer[len - 1] == '\n' || messageBuffer[len - 1] == '\r')) {
            messageBuffer[--len] = '\0';
          }

          // 디버그 출력
          char response[64];
          snprintf(response, sizeof(response), "CMD: %s\r\n", messageBuffer);
          HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);

          // 명령어 처리
          if (strcmp((char*)messageBuffer, "start") == 0) {
            enabled = 1;
            HAL_UART_Transmit(&huart2, (uint8_t*)"Motors STARTED\r\n", 16, HAL_MAX_DELAY);
          } else if (strcmp((char*)messageBuffer, "stop") == 0) {
            enabled = 0;
            baseDuty1 = baseDuty2 = baseDuty3 = baseDuty4 = 0;
            HAL_UART_Transmit(&huart2, (uint8_t*)"Motors STOPPED\r\n", 16, HAL_MAX_DELAY);
          } else if (strcmp((char*)messageBuffer, "reset") == 0) {
            roll = 0.0;
            pitch = 0.0;
            HAL_UART_Transmit(&huart2, (uint8_t*)"RESET\r\n", 13, HAL_MAX_DELAY);
          } else {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Unknown CMD\r\n", 13, HAL_MAX_DELAY);
          }

          // 버퍼 초기화
          memset(messageBuffer, 0, sizeof(messageBuffer));
          messageIndex = 0;
        }
      }
      else if (messageIndex < sizeof(messageBuffer) - 1)
      {
        messageBuffer[messageIndex++] = receivedChar;
      }
    }

    HAL_UART_Receive_IT(&huart2, rxBuffer, 1);  // 다음 문자 수신 대기
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    messageIndex = 0;
    HAL_UART_Receive_IT(&huart2, rxBuffer, 1);
  }
}
