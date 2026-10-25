/* Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * neuron6_bt.c's transmit power policy, run on a PC against a fake UART that
 * records every byte the module puts on the link: BLE transmit
 * power stays at or below 8 dBm whatever a command asks, and every other
 * vendor command is refused, whether the command is sent whole, as its type
 * byte and the rest (as the BLE host sends it), or in any other
 * pieces; data packets go through byte for byte, a data byte that looks like
 * a command's type included. */
#include <stdio.h>
#include <string.h>

#include "neuron6_bt.h"
#include "neuron6.h"
#include "neuron6_wifi.h"

/* The fake board. */
GPIO_TypeDef  fake_gpio[8];
USART_TypeDef fake_usart6;

static uint8_t  sent[4096];
static uint32_t sent_n;
static uint32_t ticks;

void HAL_GPIO_Init(GPIO_TypeDef *Port, GPIO_InitTypeDef *Init) { (void)Port; (void)Init; }
void HAL_GPIO_DeInit(GPIO_TypeDef *Port, uint32_t Pin) { (void)Port; (void)Pin; }
void HAL_GPIO_WritePin(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState State) { (void)Port; (void)Pin; (void)State; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *Init) { (void)Init; return HAL_OK; }
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t Pre, uint32_t Sub) { (void)IRQn; (void)Pre; (void)Sub; }
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn) { (void)IRQn; }
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn) { (void)IRQn; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart) { (void)huart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart) { (void)huart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{ (void)huart; (void)pData; (void)Size; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart) { (void)huart; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold) { (void)huart; (void)Threshold; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold) { (void)huart; (void)Threshold; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart) { (void)huart; return HAL_OK; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart) { (void)huart; }
uint32_t HAL_GetTick(void) { return ticks++; }
void HAL_Delay(uint32_t Delay) { ticks += Delay; }
int32_t BSP_GetTick(void) { return (int32_t)ticks++; }
int32_t BSP_WIFI_ModulePowerOn(void) { return BSP_ERROR_NONE; }
int32_t BSP_WIFI_ModulePowerOff(void) { return BSP_ERROR_NONE; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
  (void)huart;
  (void)Timeout;
  if ((sent_n + Size) <= sizeof(sent))
  {
    memcpy(&sent[sent_n], pData, Size);
    sent_n += Size;
  }
  return HAL_OK;
}

/* Checks. */
static int failures;
static int checks;

static void check(int ok, const char *what)
{
  checks++;
  if (!ok)
  {
    failures++;
    printf("FAIL  %s\n", what);
  }
}

/* What the link carried since the last call, against what it should have. */
static int carried(const uint8_t *want, uint32_t n)
{
  const int ok = (sent_n == n) && (memcmp(sent, want, n) == 0);

  if (!ok)
  {
    printf("      sent %u bytes:", (unsigned)sent_n);
    for (uint32_t i = 0; i < sent_n; i++)
    {
      printf(" %02X", sent[i]);
    }
    printf("\n");
  }
  sent_n = 0U;
  return ok;
}

/* Send `p` in the pieces `cuts` gives (lengths, 0-terminated), as one stream. */
static int32_t send_cut(const uint8_t *p, const uint32_t *cuts)
{
  int32_t  last = BSP_ERROR_NONE;
  uint32_t at   = 0U;

  for (const uint32_t *c = cuts; *c != 0U; c++)
  {
    last = BSP_BT_Send(&p[at], *c, 100U);
    at  += *c;
  }
  return last;
}

int main(void)
{
  if (BSP_BT_Init() != BSP_ERROR_NONE)
  {
    printf("FAIL  BSP_BT_Init\n");
    return 1;
  }

  /* An ordinary command, whole: unchanged. */
  {
    const uint8_t reset[] = {0x01, 0x03, 0x0C, 0x00};
    check(BSP_BT_Send(reset, sizeof reset, 100U) == BSP_ERROR_NONE, "HCI_Reset whole is accepted");
    check(carried(reset, sizeof reset), "HCI_Reset goes out as it is");
  }

  /* The way the BLE host sends: the type byte, then the rest. */
  {
    const uint8_t adv[] = {0x01, 0x0A, 0x20, 0x01, 0x01};
    const uint32_t cuts[] = {1, 4, 0};
    check(send_cut(adv, cuts) == BSP_ERROR_NONE, "a command as its type byte and the rest is accepted");
    check(carried(adv, sizeof adv), "...and goes out whole, unchanged");
  }

  /* Any pieces at all: nothing goes out before the last. */
  {
    const uint8_t adv[] = {0x01, 0x0A, 0x20, 0x01, 0x00};
    check(BSP_BT_Send(adv, 2U, 100U) == BSP_ERROR_NONE, "a command's first two bytes are taken");
    check(sent_n == 0U, "...and nothing is sent yet");
    check(BSP_BT_Send(&adv[2], 2U, 100U) == BSP_ERROR_NONE, "its next two");
    check(sent_n == 0U, "...still nothing");
    check(BSP_BT_Send(&adv[4], 1U, 100U) == BSP_ERROR_NONE, "its last byte");
    check(carried(adv, sizeof adv), "...and then the whole command goes out");
  }

  /* NXP's power command asking for 20 dBm, the loss flag set, a limit of 20. */
  {
    const uint8_t ask[]  = {0x01, 0x87, 0xFC, 0x03, 20, 0x01, 20};
    const uint8_t sent8[] = {0x01, 0x87, 0xFC, 0x03, 8, 0x00, 8};
    check(BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE, "0xFC87 asking 20 dBm is accepted");
    check(carried(sent8, sizeof sent8), "...as 8 dBm, the flag cleared, the limit 8");
  }
  {
    const uint8_t ask[]  = {0x01, 0x87, 0xFC, 0x03, 0x7F, 0x01, 0x7F};
    const uint8_t sent8[] = {0x01, 0x87, 0xFC, 0x03, 8, 0x00, 8};
    const uint32_t cuts[] = {1, 6, 0};
    check(send_cut(ask, cuts) == BSP_ERROR_NONE, "0xFC87 asking 127 dBm, as type byte and rest");
    check(carried(sent8, sizeof sent8), "...goes out as 8 dBm, the flag cleared");
  }
  {
    const uint8_t ask[] = {0x01, 0x87, 0xFC, 0x03, 4, 0x01, 6};
    const uint8_t low[] = {0x01, 0x87, 0xFC, 0x03, 4, 0x00, 6};
    check(BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE, "0xFC87 asking 4 dBm, the flag set");
    check(carried(low, sizeof low), "...keeps 4 dBm and its limit, the flag cleared");
  }
  {
    const uint8_t ask[] = {0x01, 0x87, 0xFC, 0x03, (uint8_t)-10, 0x00, (uint8_t)-10};
    check(BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE, "0xFC87 asking -10 dBm");
    check(carried(ask, sizeof ask), "...goes out as it is");
  }

  /* Other vendor commands: refused, and not a byte of them sent. */
  {
    const uint8_t cal[] = {0x01, 0x61, 0xFC, 0x02, 0x37, 0x00};
    check(BSP_BT_Send(cal, sizeof cal, 100U) == BSP_ERROR_FEATURE_NOT_SUPPORTED, "0xFC61, a calibration write, is refused");
    check(sent_n == 0U, "...and nothing of it is sent");
  }
  {
    const uint8_t cal[] = {0x01, 0xFF, 0xFC, 0x01, 0x00};
    const uint32_t cuts[] = {1, 4, 0};
    check(send_cut(cal, cuts) == BSP_ERROR_FEATURE_NOT_SUPPORTED, "0xFCFF as type byte and rest is refused");
    check(sent_n == 0U, "...and nothing of it is sent");
  }
  {
    const uint8_t reset[] = {0x01, 0x03, 0x0C, 0x00};
    check((BSP_BT_Send(reset, sizeof reset, 100U) == BSP_ERROR_NONE) && carried(reset, sizeof reset),
          "after a refusal the next command goes out whole");
  }

  /* LE Transmitter Test v4: channel, length, payload, PHY, CTE length, CTE
   * type, switching pattern length (0), power. */
  {
    const uint8_t ask[] = {0x01, 0x7B, 0x20, 0x08, 0, 37, 0, 1, 0, 0, 0, 20};
    const uint8_t at8[] = {0x01, 0x7B, 0x20, 0x08, 0, 37, 0, 1, 0, 0, 0, 8};
    check((BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE) && carried(at8, sizeof at8),
          "LE Transmitter Test v4 at 20 dBm goes out at 8");
  }
  {
    const uint8_t ask[] = {0x01, 0x7B, 0x20, 0x08, 0, 37, 0, 1, 0, 0, 0, 0x7F};
    const uint8_t at8[] = {0x01, 0x7B, 0x20, 0x08, 0, 37, 0, 1, 0, 0, 0, 8};
    check((BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE) && carried(at8, sizeof at8),
          "...at its maximum (0x7F) goes out at 8");
  }
  {
    const uint8_t ask[] = {0x01, 0x7B, 0x20, 0x08, 0, 37, 0, 1, 0, 0, 0, 0x7E};
    check((BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE) && carried(ask, sizeof ask),
          "...at its minimum (0x7E) goes out as it is");
  }
  {
    const uint8_t ask[] = {0x01, 0x7B, 0x20, 0x0A, 0, 37, 0, 1, 2, 0, 2, 0, 1, 20};
    const uint8_t at8[] = {0x01, 0x7B, 0x20, 0x0A, 0, 37, 0, 1, 2, 0, 2, 0, 1, 8};
    check((BSP_BT_Send(ask, sizeof ask, 100U) == BSP_ERROR_NONE) && carried(at8, sizeof at8),
          "...with two antenna IDs, its last parameter is the one capped");
  }

  /* Data: the type byte alone, then the header and payload in pieces, one of
   * them a single 0x01 that is data, not a command. Byte for byte. */
  {
    const uint8_t acl[] = {0x02, 0x40, 0x00, 0x06, 0x00, 0x01, 0x87, 0xFC, 0x03, 0x14, 0x01};
    const uint32_t cuts[] = {1, 4, 1, 5, 0};
    check(send_cut(acl, cuts) == BSP_ERROR_NONE, "an ACL packet in pieces is accepted");
    check(carried(acl, sizeof acl), "...and goes out byte for byte, its 0x01 and 0xFC87-like payload as data");
  }
  {
    const uint8_t reset[] = {0x01, 0x03, 0x0C, 0x00};
    check((BSP_BT_Send(reset, sizeof reset, 100U) == BSP_ERROR_NONE) && carried(reset, sizeof reset),
          "the command after it is still read as a command");
  }
  {
    const uint8_t acl[] = {0x02, 0x40, 0x20, 0x00, 0x00};
    check((BSP_BT_Send(acl, sizeof acl, 100U) == BSP_ERROR_NONE) && carried(acl, sizeof acl),
          "an empty ACL packet goes out, and ends");
  }

  /* What the host never sends. */
  {
    const uint8_t evt[] = {0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00};
    check(BSP_BT_Send(evt, sizeof evt, 100U) == BSP_ERROR_WRONG_PARAM, "an event from the host is refused");
    check(sent_n == 0U, "...and nothing is sent");
  }

  printf("%d checks, %d failed (the BLE transmit power policy)\n", checks, failures);
  return failures ? 1 : 0;
}
