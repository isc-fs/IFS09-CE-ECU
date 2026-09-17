#include "nrf24.h"

#include "main.h"
#include <string.h>

static void NRF24_BitBangDelay(void);
static HAL_StatusTypeDef NRF24_BitBangTransfer(const uint8_t *tx, uint8_t *rx, uint8_t length);

static HAL_StatusTypeDef NRF24_SpiTransfer(uint8_t value, uint8_t *rxValue);
static HAL_StatusTypeDef NRF24_ExecuteCommand(uint8_t command, const uint8_t *txData, uint8_t *rxData, uint8_t length, uint8_t *statusValue);
static HAL_StatusTypeDef NRF24_SetConfigRegister(uint8_t configValue);
static HAL_StatusTypeDef NRF24_WaitIrqAssert(uint32_t timeoutMs, uint8_t *statusValue);
static void NRF24_Settle(void);
static void NRF24_CsnDelay(void);

#define NRF24_CMD_R_RX_PAYLOAD          0x61U
#define NRF24_CMD_W_TX_PAYLOAD          0xA0U
#define NRF24_CMD_FLUSH_TX              0xE1U
#define NRF24_CMD_FLUSH_RX              0xE2U

#define NRF24_REG_CONFIG                0x00U
#define NRF24_REG_EN_AA                 0x01U
#define NRF24_REG_EN_RXADDR             0x02U
#define NRF24_REG_SETUP_AW              0x03U
#define NRF24_REG_SETUP_RETR            0x04U
#define NRF24_REG_RF_CH                 0x05U
#define NRF24_REG_RF_SETUP              0x06U
#define NRF24_REG_STATUS                0x07U
#define NRF24_REG_RX_ADDR_P0            0x0AU
#define NRF24_REG_TX_ADDR               0x10U
#define NRF24_REG_RX_PW_P0              0x11U
#define NRF24_REG_FIFO_STATUS           0x17U
#define NRF24_REG_DYNPD                 0x1CU
#define NRF24_REG_FEATURE               0x1DU

#define NRF24_CONFIG_EN_CRC             0x08U
#define NRF24_CONFIG_CRCO               0x04U
#define NRF24_CONFIG_PWR_UP             0x02U
#define NRF24_CONFIG_PRIM_RX            0x01U

#define NRF24_STATUS_RX_DR              0x40U
#define NRF24_STATUS_TX_DS              0x20U
#define NRF24_STATUS_MAX_RT             0x10U

#define NRF24_FIFO_STATUS_RX_EMPTY      0x01U

static const uint8_t g_nrf24Address[5] = { 'E', 'C', 'U', '0', '1' };

void NRF24_BusInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Bench-confirmed on this board: the SPI1 hardware peripheral read MISO as
     stuck-high (0xFF) regardless of baud rate (tried /64 and /256), while a
     plain-GPIO bit-bang on the exact same wiring reliably read the nRF24's
     real STATUS byte (0x0E). Module and wiring are proven good; the fault was
     isolated to the SPI1 peripheral's capture path on this MCU/board. SPI1 is
     no longer used at all (spi.c/spi.h removed) -- SCK/MOSI/MISO are owned
     here as plain GPIO and the protocol is driven entirely in software. */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); /* SCK idle low (mode 0) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); /* MOSI idle low */

  NRF24_SetCE(0U);
  NRF24_SetCSN(1U);
  NRF24_Settle();
}

void NRF24_SetCE(uint8_t high)
{
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void NRF24_SetCSN(uint8_t high)
{
  HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  NRF24_CsnDelay();
}

GPIO_PinState NRF24_GetIrqState(void)
{
  return HAL_GPIO_ReadPin(NRF24_IRQ_GPIO_Port, NRF24_IRQ_Pin);
}

HAL_StatusTypeDef NRF24_ReadStatusNop(uint8_t *statusValue)
{
  HAL_StatusTypeDef halStatus;
  uint8_t status = 0U;

  if (statusValue == NULL)
  {
    return HAL_ERROR;
  }

  NRF24_SetCSN(0U);
  halStatus = NRF24_SpiTransfer(0xFFU, &status);
  NRF24_SetCSN(1U);

  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  *statusValue = status;
  return HAL_OK;
}

HAL_StatusTypeDef NRF24_ReadRawNop4(uint8_t raw[4])
{
  const uint8_t tx[4] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU };

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  NRF24_SetCSN(0U);
  HAL_StatusTypeDef halStatus = NRF24_BitBangTransfer(tx, raw, 4U);
  NRF24_SetCSN(1U);
  return halStatus;
}

HAL_StatusTypeDef NRF24_ReadRegister(uint8_t reg, uint8_t *value, uint8_t *statusValue)
{
  return NRF24_ReadRegisterMulti(reg, value, 1U, statusValue);
}

HAL_StatusTypeDef NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *values, uint8_t length, uint8_t *statusValue)
{
  HAL_StatusTypeDef halStatus;
  uint8_t status = 0U;

  if ((values == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  halStatus = NRF24_ExecuteCommand((uint8_t)(reg & 0x1FU), NULL, values, length, &status);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  if (statusValue != NULL)
  {
    *statusValue = status;
  }

  return HAL_OK;
}

HAL_StatusTypeDef NRF24_WriteRegister(uint8_t reg, uint8_t value, uint8_t *statusValue)
{
  return NRF24_WriteRegisterMulti(reg, &value, 1U, statusValue);
}

HAL_StatusTypeDef NRF24_WriteRegisterMulti(uint8_t reg, const uint8_t *values, uint8_t length, uint8_t *statusValue)
{
  HAL_StatusTypeDef halStatus;
  uint8_t status = 0U;

  if ((values == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  halStatus = NRF24_ExecuteCommand((uint8_t)(0x20U | (reg & 0x1FU)), values, NULL, length, &status);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  if (statusValue != NULL)
  {
    *statusValue = status;
  }

  return HAL_OK;
}

static uint8_t s_current_config = 0xFFU;

HAL_StatusTypeDef NRF24_ApplyDefaultConfig(void)
{
  HAL_StatusTypeDef halStatus;
  uint8_t payloadWidth = NRF24_MAX_PAYLOAD_SIZE;

  /* Reset cached config to force full write-and-delay during default initialization. */
  s_current_config = 0xFFU;


  halStatus = NRF24_WriteRegister(NRF24_REG_EN_AA, 0x00U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_EN_RXADDR, 0x01U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_SETUP_AW, 0x03U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_SETUP_RETR, 0x00U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_RF_CH, 76U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_RF_SETUP, 0x06U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, g_nrf24Address, sizeof(g_nrf24Address), NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, g_nrf24Address, sizeof(g_nrf24Address), NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_RX_PW_P0, payloadWidth, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_DYNPD, 0x00U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_WriteRegister(NRF24_REG_FEATURE, 0x00U, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_FlushRx();
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_FlushTx();
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_ClearIrqFlags((uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  return NRF24_SetConfigRegister((uint8_t)(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP));
}

HAL_StatusTypeDef NRF24_RunSelfTest(NRF24_TestResult *result)
{
  HAL_StatusTypeDef halStatus;
  NRF24_TestResult localResult = {0};

  NRF24_BusInit();

  localResult.irqState = NRF24_GetIrqState();

  halStatus = NRF24_ReadStatusNop(&localResult.nopStatus);
  if (halStatus != HAL_OK)
  {
    localResult.halStatus = halStatus;
    goto done;
  }

  halStatus = NRF24_ReadRegister(0x00U, &localResult.config, &localResult.status);
  if (halStatus != HAL_OK)
  {
    localResult.halStatus = halStatus;
    goto done;
  }

  halStatus = NRF24_ReadRegister(0x05U, &localResult.rfChannelBefore, NULL);
  if (halStatus != HAL_OK)
  {
    localResult.halStatus = halStatus;
    goto done;
  }

  halStatus = NRF24_WriteRegister(0x05U, 0x2AU, &localResult.writeStatus);
  if (halStatus != HAL_OK)
  {
    localResult.halStatus = halStatus;
    goto done;
  }

  halStatus = NRF24_ReadRegister(0x05U, &localResult.rfChannelAfter, NULL);
  if (halStatus != HAL_OK)
  {
    localResult.halStatus = halStatus;
    goto done;
  }

  localResult.halStatus = HAL_OK;

done:
  if (result != NULL)
  {
    *result = localResult;
  }
  return localResult.halStatus;
}

static HAL_StatusTypeDef NRF24_SpiTransfer(uint8_t value, uint8_t *rxValue)
{
  return NRF24_BitBangTransfer(&value, rxValue, 1U);
}

/* ---- Software (bit-banged) SPI transport -----------------------------------
   Bench-confirmed: the SPI1 hardware peripheral reads MISO as stuck-high
   (0xFF) on this board regardless of baud rate (tried /64 and /256), while
   this plain-GPIO bit-bang on the exact same wiring reliably reads the
   nRF24's real STATUS byte. Module and wiring are proven good; the fault is
   isolated to the SPI1 peripheral's capture path. This is now the sole
   transport for all nRF24 traffic -- SCK/MOSI/MISO are configured as plain
   GPIO once in NRF24_BusInit() and stay that way; CSN is handled by the
   caller (NRF24_SetCSN), exactly like the old SPI1_Transfer call sites did. */
static void NRF24_BitBangDelay(void)
{
  for (volatile uint32_t i = 0U; i < 400U; ++i)
  {
    __NOP();
  }
}

static HAL_StatusTypeDef NRF24_BitBangTransfer(const uint8_t *tx, uint8_t *rx, uint8_t length)
{
  if ((tx == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  for (uint8_t i = 0U; i < length; ++i)
  {
    uint8_t txByte = tx[i];
    uint8_t rxByte = 0U;

    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, ((txByte & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      txByte = (uint8_t)(txByte << 1);
      NRF24_BitBangDelay();

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); /* rising edge: slave presents data (mode 0) */
      NRF24_BitBangDelay();
      const uint8_t sampled = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;
      rxByte = (uint8_t)((rxByte << 1) | sampled);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); /* falling edge */
      NRF24_BitBangDelay();
    }

    if (rx != NULL)
    {
      rx[i] = rxByte;
    }
  }

  return HAL_OK;
}

/* ---- Serial bench diagnosis (USART10, plain text) ------------------------ */

static void DiagAppendStr(char **out, char *end, const char *s)
{
  while ((*s != '\0') && (*out < end))
  {
    *(*out)++ = *s++;
  }
}

static void DiagAppendDec(char **out, char *end, uint32_t v)
{
  char tmp[10];
  uint8_t n = 0U;
  do
  {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while (v != 0U);
  while ((n != 0U) && (*out < end))
  {
    *(*out)++ = tmp[--n];
  }
}

static void DiagAppendHex8(char **out, char *end, uint8_t v)
{
  static const char hex[] = "0123456789ABCDEF";
  DiagAppendStr(out, end, "0x");
  if (*out < end) { *(*out)++ = hex[(v >> 4) & 0x0FU]; }
  if (*out < end) { *(*out)++ = hex[v & 0x0FU]; }
}

/* Runs a full SPI/nRF24 probe and prints a human-readable verdict over UART.
   MUST run from the SPI-owning task (radio TX), never pre-scheduler: the ~43 ms
   blocking UART TX would push boot past the 500 ms IWDG and reset-loop. The line
   buffer is static (single caller, single thread) to keep it off the task stack.
   Restores RF_CH to the operational channel (76) before returning. */
void NRF24_DiagnoseSerial(UART_HandleTypeDef *huart)
{
  static char line[640];
  char *out = line;
  char *const end = &line[sizeof(line) - 1U];

  uint8_t nop = 0xFFU;
  uint8_t raw[4] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU };
  uint8_t config = 0xFFU;
  uint8_t rfch = 0xFFU;
  uint8_t fifo = 0xFFU;
  uint8_t rb2A = 0xFFU;   /* readback of RF_CH after writing 0x2A */
  uint8_t rb15 = 0xFFU;   /* readback of RF_CH after writing 0x15 */

  if (huart == NULL)
  {
    return;
  }

  /* Known idle state: CE low, CSN high, let the module settle. */
  NRF24_BusInit();

  const uint8_t irq = (uint8_t)NRF24_GetIrqState();
  const uint8_t ce = (uint8_t)HAL_GPIO_ReadPin(NRF24_CE_GPIO_Port, NRF24_CE_Pin);
  const uint8_t csn = (uint8_t)HAL_GPIO_ReadPin(NRF24_CS_GPIO_Port, NRF24_CS_Pin);

  /* NOP-based reads bypass the 0xFF error gate, so they always show the raw
     MISO byte -- the reliable "stuck high" detector. */
  (void)NRF24_ReadStatusNop(&nop);
  (void)NRF24_ReadRawNop4(raw);

  /* Register reads (positive confirmation). These bail on 0xFF, so the vars
     stay 0xFF if MISO is dead. */
  (void)NRF24_ReadRegister(NRF24_REG_CONFIG, &config, NULL);
  (void)NRF24_ReadRegister(NRF24_REG_RF_CH, &rfch, NULL);
  (void)NRF24_ReadRegister(NRF24_REG_FIFO_STATUS, &fifo, NULL);

  /* Write-readback with two distinct patterns: the definitive SPI-alive test.
     Only a real nRF24 driving MISO can echo both values back. */
  (void)NRF24_WriteRegister(NRF24_REG_RF_CH, 0x2AU, NULL);
  (void)NRF24_ReadRegister(NRF24_REG_RF_CH, &rb2A, NULL);
  (void)NRF24_WriteRegister(NRF24_REG_RF_CH, 0x15U, NULL);
  (void)NRF24_ReadRegister(NRF24_REG_RF_CH, &rb15, NULL);
  (void)NRF24_WriteRegister(NRF24_REG_RF_CH, 76U, NULL);   /* restore operational channel */

  const uint8_t readbackOk = (uint8_t)((rb2A == 0x2AU) && (rb15 == 0x15U));
  const uint8_t stuckHigh = (uint8_t)((nop == 0xFFU) && (config == 0xFFU) &&
                                      (raw[0] == 0xFFU) && (raw[1] == 0xFFU) &&
                                      (raw[2] == 0xFFU) && (raw[3] == 0xFFU) &&
                                      (rb2A == 0xFFU) && (rb15 == 0xFFU));
  const uint8_t stuckLow = (uint8_t)((nop == 0x00U) && (raw[0] == 0x00U) &&
                                     (raw[1] == 0x00U) && (raw[2] == 0x00U) &&
                                     (raw[3] == 0x00U) && (rb2A == 0x00U) && (rb15 == 0x00U));

  DiagAppendStr(&out, end, "\r\n===== NRF24 DIAG (bit-bang SPI por software) =====\r\n");
  DiagAppendStr(&out, end, "PINS  SCK=PA5 MISO=PA6 MOSI=PA7 CSN=PB0 CE=PC5 IRQ=PC4\r\n");
  DiagAppendStr(&out, end, "SPI1 hardware DESACTIVADO: SCK/MOSI/MISO son GPIO puro (ver NRF24_BusInit)\r\n");
  DiagAppendStr(&out, end, "GPIO  IRQ=");
  DiagAppendDec(&out, end, irq);
  DiagAppendStr(&out, end, " CE=");
  DiagAppendDec(&out, end, ce);
  DiagAppendStr(&out, end, " CSN=");
  DiagAppendDec(&out, end, csn);
  DiagAppendStr(&out, end, "\r\nNOP   status=");
  DiagAppendHex8(&out, end, nop);
  DiagAppendStr(&out, end, "  raw=");
  DiagAppendHex8(&out, end, raw[0]);
  DiagAppendStr(&out, end, ",");
  DiagAppendHex8(&out, end, raw[1]);
  DiagAppendStr(&out, end, ",");
  DiagAppendHex8(&out, end, raw[2]);
  DiagAppendStr(&out, end, ",");
  DiagAppendHex8(&out, end, raw[3]);
  DiagAppendStr(&out, end, "\r\nREG   CONFIG(0x00)=");
  DiagAppendHex8(&out, end, config);
  DiagAppendStr(&out, end, " RF_CH(0x05)=");
  DiagAppendDec(&out, end, rfch);
  DiagAppendStr(&out, end, " FIFO(0x17)=");
  DiagAppendHex8(&out, end, fifo);
  DiagAppendStr(&out, end, "\r\nRDBK  wrote 0x2A->");
  DiagAppendHex8(&out, end, rb2A);
  DiagAppendStr(&out, end, "  wrote 0x15->");
  DiagAppendHex8(&out, end, rb15);
  DiagAppendStr(&out, end, "\r\nVERDICT: ");

  if (readbackOk)
  {
    DiagAppendStr(&out, end, "OK - nRF24 responde correctamente por bit-bang (readback OK)");
  }
  else if (stuckHigh)
  {
    DiagAppendStr(&out, end, "FALLO: MISO SIEMPRE ALTO (0xFF) incluso por bit-bang.");
    DiagAppendStr(&out, end, "\r\n         Esto SI es electrico: revisa 3V3 + cap 10uF en VCC del modulo,");
    DiagAppendStr(&out, end, "\r\n         cable MISO(PA6), CSN(PB0) y masa comun.");
  }
  else if (stuckLow)
  {
    DiagAppendStr(&out, end, "FALLO: MISO SIEMPRE BAJO (0x00) - MISO a masa o modulo sin alimentar.");
  }
  else
  {
    DiagAppendStr(&out, end, "INESTABLE: readback no coincide - revisa alimentacion o soldaduras.");
  }
  DiagAppendStr(&out, end, "\r\n=========================\r\n");

  (void)HAL_UART_Transmit(huart, (uint8_t *)line, (uint16_t)(out - line), 200U);
}

HAL_StatusTypeDef NRF24_ClearIrqFlags(uint8_t irqMask)
{
  return NRF24_WriteRegister(NRF24_REG_STATUS, (uint8_t)(irqMask & (NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)), NULL);
}

HAL_StatusTypeDef NRF24_FlushRx(void)
{
  return NRF24_ExecuteCommand(NRF24_CMD_FLUSH_RX, NULL, NULL, 0U, NULL);
}

HAL_StatusTypeDef NRF24_FlushTx(void)
{
  return NRF24_ExecuteCommand(NRF24_CMD_FLUSH_TX, NULL, NULL, 0U, NULL);
}

HAL_StatusTypeDef NRF24_EnterRxMode(void)
{
  HAL_StatusTypeDef halStatus = NRF24_SetConfigRegister((uint8_t)(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO | NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX));
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  NRF24_SetCE(1U);
  return HAL_OK;
}

HAL_StatusTypeDef NRF24_SendPayload(const uint8_t *payload, uint8_t length, uint32_t timeoutMs)
{
  HAL_StatusTypeDef halStatus;
  uint8_t txBuffer[NRF24_MAX_PAYLOAD_SIZE] = {0};
  uint8_t status = 0U;

  if ((payload == NULL) || (length == 0U) || (length > NRF24_MAX_PAYLOAD_SIZE))
  {
    return HAL_ERROR;
  }

  (void)memcpy(txBuffer, payload, length);

  NRF24_SetCE(0U);
  halStatus = NRF24_SetConfigRegister((uint8_t)(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP));
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_ClearIrqFlags((uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_FlushTx();
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  halStatus = NRF24_ExecuteCommand(NRF24_CMD_W_TX_PAYLOAD, txBuffer, NULL, NRF24_MAX_PAYLOAD_SIZE, &status);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  NRF24_SetCE(1U);
  for (volatile uint32_t i = 0U; i < 800U; ++i)
  {
    __NOP();
  }
  NRF24_SetCE(0U);

  halStatus = NRF24_WaitIrqAssert(timeoutMs, &status);
  (void)NRF24_ClearIrqFlags((uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));
  (void)NRF24_SetConfigRegister((uint8_t)(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP));

  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  if ((status & NRF24_STATUS_TX_DS) != 0U)
  {
    return HAL_OK;
  }

  if ((status & NRF24_STATUS_MAX_RT) != 0U)
  {
    (void)NRF24_FlushTx();
    return HAL_TIMEOUT;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef NRF24_ReadRxPayload(uint8_t *payload, uint8_t length)
{
  if ((payload == NULL) || (length == 0U) || (length > NRF24_MAX_PAYLOAD_SIZE))
  {
    return HAL_ERROR;
  }

  return NRF24_ExecuteCommand(NRF24_CMD_R_RX_PAYLOAD, NULL, payload, length, NULL);
}

static HAL_StatusTypeDef NRF24_ExecuteCommand(uint8_t command, const uint8_t *txData, uint8_t *rxData, uint8_t length, uint8_t *statusValue)
{
  HAL_StatusTypeDef halStatus;
  uint8_t tx[NRF24_MAX_PAYLOAD_SIZE + 1U] = {0U};
  uint8_t rx[NRF24_MAX_PAYLOAD_SIZE + 1U] = {0U};
  uint8_t status = 0U;

  if (length > NRF24_MAX_PAYLOAD_SIZE)
  {
    return HAL_ERROR;
  }

  tx[0] = command;
  for (uint8_t index = 0U; index < length; index++)
  {
    tx[index + 1U] = (txData != NULL) ? txData[index] : 0xFFU;
  }

  NRF24_SetCSN(0U);
  halStatus = NRF24_BitBangTransfer(tx, rx, (uint8_t)(length + 1U));
  NRF24_SetCSN(1U);

  status = rx[0];

  if ((halStatus == HAL_OK) && (status == 0xFFU))
  {
    return HAL_ERROR;
  }

  if ((halStatus == HAL_OK) && (rxData != NULL))
  {
    for (uint8_t index = 0U; index < length; index++)
    {
      rxData[index] = rx[index + 1U];
    }
  }

  if (statusValue != NULL)
  {
    *statusValue = status;
  }

  return halStatus;
}

static HAL_StatusTypeDef NRF24_SetConfigRegister(uint8_t configValue)
{
  if (configValue == s_current_config)
  {
    return HAL_OK;
  }

  HAL_StatusTypeDef halStatus = NRF24_WriteRegister(NRF24_REG_CONFIG, configValue, NULL);
  if (halStatus != HAL_OK)
  {
    return halStatus;
  }

  /* nRF24 datasheet Tpd2stby: >=1.5 ms delay only needed when transitioning 
   * PWR_UP from 0 to 1. Skip delay if PWR_UP was already high.
   * Under -O0 (Debug), a 100,000 loop takes ~1.5 million cycles (approx 3 ms at 480 MHz). */
  if (((s_current_config & NRF24_CONFIG_PWR_UP) == 0U) && ((configValue & NRF24_CONFIG_PWR_UP) != 0U))
  {
    for (volatile uint32_t nop_i = 0U; nop_i < 100000U; ++nop_i) { __NOP(); }
  }

  s_current_config = configValue;
  return HAL_OK;
}

static HAL_StatusTypeDef NRF24_WaitIrqAssert(uint32_t timeoutMs, uint8_t *statusValue)
{
  uint32_t startTick = HAL_GetTick();
  uint8_t status = 0U;

  do
  {
    if (NRF24_ReadStatusNop(&status) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (status == 0xFFU)
    {
      return HAL_ERROR;
    }

    if ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT | NRF24_STATUS_RX_DR)) != 0U)
    {
      if (statusValue != NULL)
      {
        *statusValue = status;
      }

      return HAL_OK;
    }

    if (NRF24_GetIrqState() == GPIO_PIN_RESET)
    {
      /* IRQ should go low when any event bit is set. Read STATUS again so we
         can trust the latched flags even if the IRQ pulse is short or noisy. */
      if (NRF24_ReadStatusNop(&status) != HAL_OK)
      {
        return HAL_ERROR;
      }
      if (status == 0xFFU)
      {
        return HAL_ERROR;
      }

      if (statusValue != NULL)
      {
        *statusValue = status;
      }

      if ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT | NRF24_STATUS_RX_DR)) != 0U)
      {
        return HAL_OK;
      }
    }
  } while ((HAL_GetTick() - startTick) < timeoutMs);

  if (NRF24_ReadStatusNop(&status) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (status == 0xFFU)
  {
    return HAL_ERROR;
  }

  if (statusValue != NULL)
  {
    *statusValue = status;
  }

  if ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT | NRF24_STATUS_RX_DR)) == 0U)
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

static void NRF24_Settle(void)
{
  for (volatile uint32_t i = 0U; i < 200000U; ++i)
  {
    __NOP();
  }
}

static void NRF24_CsnDelay(void)
{
  for (volatile uint32_t i = 0U; i < 200U; ++i)
  {
    __NOP();
  }
}
