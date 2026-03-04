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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeg_ads1299.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "stm32l4xx_ll_adc.h"
#include "common_header.h"
#include "afe4404.h"
#include "afe4404_debug.h"
#include "afe_measure.h"
#include "ppg_task.h"
#include "nb_proto_v01.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BM64_MFB_PORT GPIOB
#define BM64_MFB_PIN  GPIO_PIN_2

#define LED_PORT      GPIOA
#define LED_PIN       GPIO_PIN_5

#define BTN_PIN       GPIO_PIN_13
#define BTN_PORT      GPIOC

/* UART */
extern UART_HandleTypeDef huart1;   // BM64 ?  ?
extern UART_HandleTypeDef huart2;   // 로그 출력
extern UART_HandleTypeDef huart3;   // PC ?��?��(USART3)
extern I2C_HandleTypeDef hi2c1;    // AFE4404 I2C

/* UART Protocol */
#define WAKEUP 0x00
#define START  0xAA

/* Commands / Events */
#define CMD_MMI_ACTION   0x02
#define CMD_MUSIC_CTRL   0x04
#define EVT_CMD_ACK      0x00
#define EVT_BTM_STATUS   0x01

/* BLE Transparent/LE Data (UART Command Set) */
#define CMD_SEND_SPP_IAP_LE_DATA   0x12  /* Send_SPP/iAP_Or_LE_Data */
#define EVT_REPORT_SPP_IAP_LE_DATA 0x22  /* Report_SPP/iAP/LE_Data */

/* Actions */
#define ACT_PWR_ON_P     0x51
#define ACT_PWR_ON_R     0x52
#define ACT_PWR_OFF_P    0x53
#define ACT_PWR_OFF_R    0x54

/* Pairing / Discoverable (MMI_Action) */
#define ACT_PAIR_FAST    0x5D  /* Fast Enter Pairing Mode (from Non-off Mode) */
#define ACT_EXIT_PAIR    0x6B  /* Exit Pairing Mode */

#define DATA_BASE_INDEX  0x00

/* Timing */
#define T_PW_MS          500
#define ACK_TO_MS        2000
#define STATUS_TO_MS     6000
#define BTN_DEB_MS       200

/* =========================
 * AFE4404 UART CSV capture
 *  - Button long-press starts capture
 *  - Capture duration can be changed by editing g_meas_duration_ms
 * ========================= */
#define AFE_I2C_ADDR_7B        0x58    /* AFE4404 I2C slave address (hex) */
#define MEAS_HOLD_MS           800     /* long-press threshold to start capture */
static uint32_t g_meas_duration_ms = 2000; /* <-- change this to adjust capture time */

static volatile uint8_t  g_meas_active = 0;
static volatile uint32_t g_meas_t0_ms = 0;
static volatile uint32_t g_meas_sent = 0;
static volatile uint32_t g_meas_pending = 0; /* incremented in AFE_RDY EXTI, serviced in main loop */
static volatile uint8_t  g_btn_latch = 0;
static volatile uint32_t g_btn_press_ms = 0;

/* AFE4404 data registers (24-bit) */
#define REG_LED1VAL   0x2E
#define REG_ALED1VAL  0x2D
#define REG_LED2VAL   0x2C
#define REG_ALED2VAL  0x2B


/* OFF event (?????측된 ?  벤트) */
#define EVT_POWER_EVENT  0x32
#define OFF_EVT_TO_MS    1500

/* RX buffer */
#define RX_SZ 512
static volatile uint8_t  rx[RX_SZ];
static volatile uint16_t wptr;
static uint8_t rx1;


/* PC(UART3) RX buffer */
#define PC_RX_SZ 512
static volatile uint8_t  pc_rx[PC_RX_SZ];
static volatile uint16_t pc_wptr;
static uint8_t pc_rx1;

#define BM64_MAX_PAYLOAD 64

static bool uart_send(uint8_t op, const uint8_t* prm, uint16_t plen);
static volatile uint8_t  g_uart1_tx_busy = 0;
/* USART3 (PC) non-blocking TX ring buffer */
#define PC_TX_RING_SZ 1024
static volatile uint8_t  g_uart3_tx_busy = 0;
static uint8_t           pc_tx_ring[PC_TX_RING_SZ];
static volatile uint16_t pc_tx_wptr = 0;
static volatile uint16_t pc_tx_rptr = 0;
static uint8_t           pc_tx_chunk[256];
static uint16_t          pc_tx_chunk_len = 0;

static void pc_tx_kick(void);
static void pc_tx_flush(void)
{
  __disable_irq();
  pc_tx_wptr = 0;
  pc_tx_rptr = 0;
  __enable_irq();
}

static uint16_t pc_tx_ring_count(void)
{
  uint16_t w = pc_tx_wptr;
  uint16_t r = pc_tx_rptr;
  if (w >= r) return (uint16_t)(w - r);
  return (uint16_t)(PC_TX_RING_SZ - r + w);
}

static void pc_tx_ring_push(const uint8_t *data, uint16_t n)
{
  if (!data || !n) return;
  __disable_irq();
  while (n--) {
    uint16_t next = (uint16_t)((pc_tx_wptr + 1) % PC_TX_RING_SZ);
    if (next == pc_tx_rptr) {
      /* overflow: drop remaining */
      break;
    }
    pc_tx_ring[pc_tx_wptr] = *data++;
    pc_tx_wptr = next;
  }
  __enable_irq();
}

static uint16_t pc_tx_ring_pop_to_chunk(void)
{
  uint16_t cnt = 0;
  __disable_irq();
  while ((pc_tx_rptr != pc_tx_wptr) && (cnt < sizeof(pc_tx_chunk))) {
    pc_tx_chunk[cnt++] = pc_tx_ring[pc_tx_rptr];
    pc_tx_rptr = (uint16_t)((pc_tx_rptr + 1) % PC_TX_RING_SZ);
  }
  __enable_irq();
  return cnt;
}

static void pc_tx_kick(void)
{
  if (g_uart3_tx_busy) return;
  if (pc_tx_rptr == pc_tx_wptr) return;
  pc_tx_chunk_len = pc_tx_ring_pop_to_chunk();
  if (!pc_tx_chunk_len) return;
  g_uart3_tx_busy = 1;
  if (HAL_UART_Transmit_IT(&huart3, pc_tx_chunk, pc_tx_chunk_len) != HAL_OK) {
    g_uart3_tx_busy = 0;
  }
}

static uint8_t           g_uart1_tx_buf[MAX_BLE_BUFFER+20];
static uint16_t          g_uart1_tx_len = 0;

/* =============================================================
 * BLE Data Streaming (ADC 12bit x10ch @256Hz)
 *  - Phone(App) writes START/STOP via BLE -> BM64 UART event(0x22)
 *  - STM32 samples ADC @256Hz, buffers samples
 *  - Frames are built (FRAME_SAMPLES) then sent to BM64 via 0x12
 *  - 0x12 supports fragmentation; we use 20-byte chunks for max compatibility
 * ============================================================= */
#define ADC_CH_COUNT        10
#define ADC_FS_HZ           256
#define FRAME_SAMPLES       16      /* 16 samples -> 62.5ms/frame (within 50~200ms) */
//#define BLE_CHUNK_MAX       20      /* safe default: works even if MTU=23 */
#define BLE_CHUNK_MAX       180      /* safe default: works even if MTU=23 */

/* Ring buffer of raw samples (uint16_t[10]) */
#define SAMPLE_RING_CAP     512     /* 512 samples ~= 2 seconds @256Hz */

typedef struct {
  uint16_t v[ADC_CH_COUNT];
} adc_sample_t;

static volatile uint8_t  g_do_256hz = 0;
static volatile uint8_t  g_stream_on = 0;
static volatile uint8_t  g_le_ch_index = 0; /* channel index from BM64 (0x22) */
/* ?   FIX: ch=0x00?   ?  ?  ?   ?   ?  ?  ?????????? "0?  ????? 미확?  " ??????    ? ? */
static volatile uint8_t  g_le_ch_valid = 0; /* 채널 ?  ?  ?  ????? ?   번이?  ?   ?  ?  ?  ?  ????? */

/* BLE link heuristic (minimal / non-invasive)
 * - BM64 LE connect status event(0x32) is already used in this project as power/off event.
 * - To avoid breaking existing logic, we treat "BLE is ready" as:
 *     we have received at least one EVT_REPORT_SPP_IAP_LE_DATA(0x22) recently.
 * - This prevents 0x12 ACK timeouts when phone is not connected.
 */
static volatile uint8_t  g_ble_ready = 0;
static volatile uint32_t g_ble_last_rx_ms = 0;

static adc_sample_t s_ring[SAMPLE_RING_CAP];
static volatile uint16_t s_r_w = 0; /* write index */
static volatile uint16_t s_r_r = 0; /* read index */
static volatile uint32_t s_r_drop = 0;

static uint16_t g_seq = 0; /* frame sequence */

/* TX state for one frame fragmentation */
typedef struct {
  uint8_t  active;
  uint16_t total;
  uint16_t sent;
  uint8_t  buf[4 + 4 + 15*FRAME_SAMPLES];
  /* layout: [SEQ(2)][NS(1)][FLAGS(1)] + packed(15*N) */
} ble_tx_state_t;

static ble_tx_state_t g_tx;

int32_t gv_i32_fnirs_save_buf[FNIRS_TX_PAGE_MAX][4];
int32_t gv_i32_fnirs_tx_buf[FNIRS_TX_PAGE_MAX];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* AFE4404 handles */
static afe4404_t g_afe;
static afe_measure_t g_afe_meas;

#define VREFINT_mV       1210UL
volatile uint8_t g_do_100ms = 0;
uint32_t vref_cal = 0;

volatile uint32_t g_bm64_rx_cnt = 0;
volatile uint32_t g_bm64_rx_drop = 0;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2; // 모니?  ???
// BM64?   보통 huart1??? ?  ??? ?  ?   중일 ????  ?  ?   ?

static ppg_task_t g_ppg;
volatile uint32_t g_afe_rdy_cnt = 0;

/* EEG (ADS1299) handle */
static eeg_ads1299_t g_eeg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static bool parse(uint8_t* op, uint8_t* prm, uint16_t* plen);
static bool bm64_send_le_data_fragment(uint8_t type, uint16_t total_len,
                                      const uint8_t* payload, uint16_t payload_len);

static void TP01(bool on);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* -------------------- Button EXTI -------------------- */
static volatile bool be = false;
static uint32_t bt = 0;
static bool test_port_value = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI_DRDY_Pin) { // PB0 SPI_DRDY (EEG ADS1299)
        eeg_ads1299_on_drdy_isr(&g_eeg);
    }

    if (GPIO_Pin == GPIO_PIN_9) { // PC9 AFE_RDY
        ppg_task_on_rdy_isr(&g_ppg);
        g_afe_rdy_cnt++;
//        if(test_port_value== 0)
//		{
//			test_port_value = 1;
//		}
//		else
//		{
//			test_port_value = 0;
//		}
//		TP01(test_port_value);
    }

    // 버튼?   EXTI?  ???, 버튼 ???   ?  기서 처리 ????
    if (GPIO_Pin == BTN_PIN)
      {
        if (!g_btn_latch) { g_btn_latch = 1; g_btn_press_ms = HAL_GetTick(); }
        uint32_t n = HAL_GetTick();
        if (n - bt > BTN_DEB_MS)
        {
          bt = n;
          be = true;
        }

      }
}

static bool app_ble_send(const uint8_t *data, uint16_t len)
{
    // TODO: ?  기서 BM64 BLE ?  ?   ?  ?  ?? ?  결해?   ?  .
    // ?  ) return bm64_send_le_data(data, len);

    // ?  ?  : 컴파?  ?? ?  과시?  ??(?  ?   ?   ?  )
    (void)data;
    (void)len;
    return true;
}

/* -------------------- LOG -------------------- */
static void log_uart2(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 50);
}

static void logf(const char *fmt, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 50);
}

/* -------------------- AFE4404 I2C helpers (24-bit regs) -------------------- */
static HAL_StatusTypeDef afe_rd24(uint8_t reg, uint32_t* out)
{
  uint8_t b[3] = {0};
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(AFE_I2C_ADDR_7B << 1),
                                         reg, I2C_MEMADD_SIZE_8BIT, b, 3, 50);
  if (st != HAL_OK) return st;
  *out = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
  return HAL_OK;
}

static void afe_read_and_csv_send(void)
{
  uint32_t led1=0, aled1=0, led2=0, aled2=0;
  if (afe_rd24(REG_LED1VAL, &led1) != HAL_OK) return;
  if (afe_rd24(REG_ALED1VAL, &aled1) != HAL_OK) return;
  if (afe_rd24(REG_LED2VAL, &led2) != HAL_OK) return;
  if (afe_rd24(REG_ALED2VAL, &aled2) != HAL_OK) return;

  uint32_t t = HAL_GetTick() - g_meas_t0_ms;
  /* CSV: ms,LED1VAL,ALED1VAL,LED2VAL,ALED2VAL */
  logf("%lu,%lu,%lu,%lu,%lu\r\n", (unsigned long)t,
       (unsigned long)led1, (unsigned long)aled1,
       (unsigned long)led2, (unsigned long)aled2);
  g_meas_sent++;
}

static void meas_start(void)
{
  g_meas_active = 1;
  g_meas_t0_ms  = HAL_GetTick();
  g_meas_sent   = 0;
  g_meas_pending = 0;

  log_uart2("ms,LED1VAL,ALED1VAL,LED2VAL,ALED2VAL\r\n");
  logf("#MEAS_START,duration_ms=%lu\r\n", (unsigned long)g_meas_duration_ms);
}

static void meas_stop(void)
{
  g_meas_active = 0;
  logf("#MEAS_STOP,sent=%lu\r\n", (unsigned long)g_meas_sent);
}

/* === AFE4404 RESETZ PWDN + MCO Hi-Z (button long-press toggle) === */
static uint8_t g_afe_pwdn = 0; /* 0=RUNNING, 1=PWDN */

static void afe_mco_hiz(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* MCO1 is on PA8 on STM32L476 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /* Hi-Z */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void afe_mco_restore_8mhz(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Same config used at boot */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);
}

static void afe_enter_pwdn_flow(void)
{
  /* stop CSV capture if running */
  if (g_meas_active) meas_stop();

  /* Optional: stop timing engine cleanly (writes are valid only before PWDN) */
  /* If you have a dedicated API, use it. Otherwise, leave as-is. */
  /* afe4404_write24_raw(&g_afe, 0x1E, 0x000003); */
  HAL_Delay(2);

  /* External clock Hi-Z to avoid CLK contention while AFE is in PWDN */
  afe_mco_hiz();
  HAL_Delay(1);

  /* RESETZ low >=200us => PWDN (use 1ms) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); /* AFE_RESETZ */
  HAL_Delay(1);
}

static void afe_exit_pwdn_and_start_flow(void)
{
  /* Release PWDN */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); /* AFE_RESETZ */
  HAL_Delay(5);

  /* Restore external clock */
  afe_mco_restore_8mhz();
  HAL_Delay(1);

  /* HW reset (short pulse inside driver) */
  afe4404_hw_reset();
  HAL_Delay(2);

  /* Re-apply full init sequence (same as boot) */
  afe4404_apply_regset_from_working_dump(&g_afe);
  afe_measure_init(&g_afe_meas, &g_afe, &huart2, 100);
  afe_measure_start(&g_afe_meas);
}

static void afe_pwdn_toggle_on_longpress(void)
{
  if (g_afe_pwdn == 0) {
    afe_enter_pwdn_flow();
    g_afe_pwdn = 1;
    logf("[AFE] ENTER PWDN\r\n");
  } else {
    afe_exit_pwdn_and_start_flow();
    g_afe_pwdn = 0;
    logf("[AFE] EXIT PWDN -> RUNNING\r\n");
  }
}

void afe_task_stop(void)
{
		afe_enter_pwdn_flow();
		g_afe_pwdn = 1;
		logf("[AFE] ENTER PWDN\r\n");

}
void afe_task_start(void)
{
	afe_exit_pwdn_and_start_flow();
	g_afe_pwdn = 0;
	logf("[AFE] EXIT PWDN -> RUNNING\r\n");
}
/* === END OF AFE PWDN SECTION === */


/* -------------------- BM64 RESET -------------------- */
#define BM64_RST_PORT   GPIOB
#define BM64_RST_PIN    GPIO_PIN_1

static void BM64_ResetPulse(void)
{
  HAL_Delay(100);
  HAL_GPIO_WritePin(BM64_RST_PORT, BM64_RST_PIN, GPIO_PIN_RESET); // Active Low
  HAL_Delay(20);
  HAL_GPIO_WritePin(BM64_RST_PORT, BM64_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(200);
}

/* -------------------- RX handling -------------------- */
static void rx_start(void)
{
  HAL_UART_Receive_IT(&huart1, &rx1, 1);
}

static void rx_clear(void)
{
  __disable_irq();
  memset((void*)rx, 0, sizeof(rx));
  wptr = 0;
  __enable_irq();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hu)
{
  if (hu->Instance == USART1)
  {
    g_bm64_rx_cnt++;
    if (wptr < RX_SZ) {
      rx[wptr++] = rx1;
    } else {
      g_bm64_rx_drop++;
    }
    HAL_UART_Receive_IT(&huart1, &rx1, 1);
  }
  else if (hu->Instance == USART3)
  {
    if (pc_wptr < PC_RX_SZ) {
      pc_rx[pc_wptr++] = pc_rx1;
    }
    HAL_UART_Receive_IT(&huart3, &pc_rx1, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hu)
{
  if (hu->Instance == USART1) {
    g_uart1_tx_busy = 0;
  } else if (hu->Instance == USART3) {
    g_uart3_tx_busy = 0;
    pc_tx_kick();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *hu)
{
  if (hu->Instance == USART1) {
    g_uart1_tx_busy = 0;
  } else if (hu->Instance == USART3) {
    g_uart3_tx_busy = 0;
    pc_tx_kick();
  }
}

/* UART RX ?  ?  기화( ? ??   ?   복구?  ) */
static void bm64_uart_resync(void)
{
  rx_clear();
  HAL_UART_AbortReceive_IT(&huart1);
  HAL_UART_Receive_IT(&huart1, &rx1, 1);
}


/* ==================== NeoBrain v0.1 parser (UART3 + BLE common) ==================== */

/* v0.1 CMD (per spec) */
#define NB_CMD_REQ_DEVICE_BEACON   0x01
#define NB_CMD_RES_DEVICE_BEACON   0x02
#define NB_CMD_REQ_OP_START        0x10
#define NB_CMD_RES_OP_START        0x11
#define NB_CMD_REQ_OP_STOP         0x12
#define NB_CMD_RES_OP_STOP         0x13
#define NB_CMD_BC_EEG_DATA         0x21
#define NB_CMD_BC_EEG_END          0x22
#define NB_CMD_BC_FNIRS_DATA       0x31
#define NB_CMD_BC_FNIRS_END        0x32
#define NB_CMD_BC_TDCS_DATA 		0x41
#define NB_CMD_BC_TDCS_END        	0x42

#define NB_SUBCMD_EEG				0x01
#define NB_SUBCMD_FNIRS				0x02
#define NB_SUBCMD_TDCS				0x04


/* Device Status bit field: 0x00 idle, 0xFF error, 0x0X = operating flags
 *  bit0 EEG, bit1 fNIRS, bit2 tDCS
 */
static volatile uint8_t  g_nb_dev_status = 0x00;
static volatile uint8_t  g_nb_eeg_on = 0;
static volatile uint32_t g_nb_seq = 0;
static volatile uint8_t  g_nb_fnirs_on = 0;
static volatile uint32_t g_nb_seq_fnirs = 0;
static volatile uint32_t g_nb_pack_fnirs = 0;

static volatile uint8_t  g_nb_tdcs_on = 0;


static uint32_t g_nb_last_beacon_ms = 0;
static uint32_t g_nb_last_eeg_ms = 0;

typedef enum { NB_CH_PC = 0, NB_CH_BLE = 1 } nb_ch_t;

typedef struct { nb_ch_t ch; } nb_cb_ctx_t;

static nb_parser_t g_nb_parser_pc;
static nb_parser_t g_nb_parser_ble;
static nb_cb_ctx_t g_nb_ctx_pc  = { .ch = NB_CH_PC };
static nb_cb_ctx_t g_nb_ctx_ble = { .ch = NB_CH_BLE };

static void pc_rx_start(void)
{
  pc_wptr = 0;
  HAL_UART_Receive_IT(&huart3, &pc_rx1, 1);
}

static void pc_rx_clear(void)
{
  __disable_irq();
  memset((void*)pc_rx, 0, sizeof(pc_rx));
  pc_wptr = 0;
  __enable_irq();
}




static bool bm64_send_le_bytes(const uint8_t *data, uint16_t len);

static void nb_send_on_channel(nb_ch_t ch, const uint8_t *bytes, uint16_t n)
{
  if (!bytes || !n) return;
  if (ch == NB_CH_PC) {
    pc_tx_ring_push(bytes, n);
    pc_tx_kick();
    return;
  }
  /* BLE(TDT) is optional; skip sending when phone is not connected to avoid
   * 0x12 ACK timeouts and to keep existing BM64 audio/control stable.
   */
  if (!g_ble_ready) {
	logf("[NB->BLE] SKIP g_ble_ready=0 n=%u\r\n", n);
    return;
  }
//  logf("[NB->BLE] TRY n=%u le_valid=%u le_ch=%02X\r\n", n, g_le_ch_valid, g_le_ch_index);
  (void)bm64_send_le_bytes(bytes, n);
}

static void nb_send_both(const uint8_t *bytes, uint16_t n)
{
  nb_send_on_channel(NB_CH_PC, bytes, n);
  nb_send_on_channel(NB_CH_BLE, bytes, n);
}

static void nb_send_beacon(void)
{
  /* CMD 0x01 REQ Device Beecon (DEV->APP)
   * Data(4): Device Type(1), Version HI(1), Version LO(1), Device Status(1)
   */
  uint8_t data[4];
  data[0] = 0x01; /* Device Type: EEG */
  data[1] = 0x00; /* Version HI */
  data[2] = 0x01; /* Version LO (v0.1) */
  data[3] = g_nb_dev_status;

  uint8_t out[2 + 1 + 2 + 4 + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_REQ_DEVICE_BEACON, data, sizeof(data), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}

static void nb_send_eeg_dummy_frame(void)
{
  /* CMD 0x21 Broadcast EEG Data
   * Field list (  c8 \84\9c \ed\91\9c \ea80\eb0\98): Reserved(1) + SeqNum(4) + PackageNum(2) + (CH1..CH4 2B)*20
   * Total = 1+4+2+160 = 167 bytes
   */
  uint8_t payload[167];
  uint16_t k = 0;

  payload[k++] = 0x00;
  payload[k++] = (uint8_t)((g_nb_seq >> 24) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq >> 16) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq >>  8) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq >>  0) & 0xFF);

  uint16_t pkg_num = EEG_TX_PAGE_MAX;
  payload[k++] = (uint8_t)((pkg_num >> 8) & 0xFF);
  payload[k++] = (uint8_t)(pkg_num & 0xFF);

  for (int p = 0; p < EEG_TX_PAGE_MAX; p++) {
    for (int ch = 0; ch < 4; ch++) {
      uint16_t v = (uint16_t)((g_nb_seq + p + ch) & 0x0FFF);
      payload[k++] = (uint8_t)((v >> 8) & 0xFF);
      payload[k++] = (uint8_t)(v & 0xFF);
    }
  }

  uint8_t out[2 + 1 + 2 + sizeof(payload) + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_EEG_DATA, payload, sizeof(payload), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}

static void nb_send_eeg_end(uint32_t total_pkg)
{
  /* CMD 0x22 Brodcat EEG Operation End: Total PackageNum(4) + Reserved(1) */
  uint8_t data[5];
  data[0] = (uint8_t)((total_pkg >> 24) & 0xFF);
  data[1] = (uint8_t)((total_pkg >> 16) & 0xFF);
  data[2] = (uint8_t)((total_pkg >>  8) & 0xFF);
  data[3] = (uint8_t)((total_pkg >>  0) & 0xFF);
  data[4] = 0x00;

  uint8_t out[2 + 1 + 2 + 5 + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_EEG_END, data, sizeof(data), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}


static void nb_send_fnirs_dummy_frame(void)
{
  /* CMD 0x21 Broadcast EEG Data
   * Field list (  c8 \84\9c \ed\91\9c \ea80\eb0\98): Reserved(1) + SeqNum(4) + PackageNum(2) + (CH1..CH4 2B)*20
   * Total = 1+4+2+160 = 167 bytes
   */
  uint8_t payload[167];
  uint16_t k = 0;

  payload[k++] = 0x00;
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >> 24) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >> 16) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >>  8) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >>  0) & 0xFF);

  uint16_t pkg_num = FNIRS_TX_PAGE_MAX;
  payload[k++] = (uint8_t)((pkg_num >> 8) & 0xFF);
  payload[k++] = (uint8_t)(pkg_num & 0xFF);

  for (int p = 0; p < FNIRS_TX_PAGE_MAX; p++) {
    for (int ch = 0; ch < 4; ch++) {
      uint16_t v = (uint16_t)((g_nb_seq_fnirs + p + ch) & 0x0FFF);
      payload[k++] = (uint8_t)((v >> 24) & 0xFF);
      payload[k++] = (uint8_t)((v >> 16) & 0xFF);
      payload[k++] = (uint8_t)((v >> 8) & 0xFF);
      payload[k++] = (uint8_t)(v & 0xFF);
    }
  }

  uint8_t out[2 + 1 + 2 + sizeof(payload) + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_FNIRS_DATA, payload, sizeof(payload), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}

static void nb_send_fnirs_frame(void)
{
  /* CMD 0x21 Broadcast EEG Data
   * Field list (  c8 \84\9c \ed\91\9c \ea80\eb0\98): Reserved(1) + SeqNum(4) + PackageNum(2) + (CH1..CH4 2B)*20
   * Total = 1+4+2+160 = 167 bytes
   */
  uint8_t payload[167];
  uint16_t k = 0;

  payload[k++] = 0x00;
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >> 24) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >> 16) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >>  8) & 0xFF);
  payload[k++] = (uint8_t)((g_nb_seq_fnirs >>  0) & 0xFF);

  uint16_t pkg_num = FNIRS_TX_PAGE_MAX;
  payload[k++] = (uint8_t)((pkg_num >> 8) & 0xFF);
  payload[k++] = (uint8_t)(pkg_num & 0xFF);

  for (int p = 0; p < FNIRS_TX_PAGE_MAX; p++) {
    for (int ch = 0; ch < 4; ch++) {
//      uint16_t v = (uint16_t)((g_nb_seq_fnirs + p + ch) & 0x0FFF);
      payload[k++] = (uint8_t)((gv_i32_fnirs_save_buf[p][ch] >> 24) & 0xFF);
      payload[k++] = (uint8_t)((gv_i32_fnirs_save_buf[p][ch] >> 16) & 0xFF);
      payload[k++] = (uint8_t)((gv_i32_fnirs_save_buf[p][ch] >> 8) & 0xFF);
      payload[k++] = (uint8_t)((gv_i32_fnirs_save_buf[p][ch] >> 0) & 0xFF);
    }
  }

  uint8_t out[2 + 1 + 2 + sizeof(payload) + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_FNIRS_DATA, payload, sizeof(payload), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}


static void nb_send_fnirs_end(uint32_t total_pkg)
{
  /* CMD 0x22 Brodcat EEG Operation End: Total PackageNum(4) + Reserved(1) */
  uint8_t data[5];
  data[0] = (uint8_t)((total_pkg >> 24) & 0xFF);
  data[1] = (uint8_t)((total_pkg >> 16) & 0xFF);
  data[2] = (uint8_t)((total_pkg >>  8) & 0xFF);
  data[3] = (uint8_t)((total_pkg >>  0) & 0xFF);
  data[4] = 0x00;

  uint8_t out[2 + 1 + 2 + 5 + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_FNIRS_END, data, sizeof(data), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}

static void nb_send_tdcs_oper(uint8_t time)
{
  /* CMD 0x22 Brodcat EEG Operation End: Total PackageNum(4) + Reserved(1) */
  uint8_t data[2];
  data[0] = time;
  data[1] = 0x00;

  uint8_t out[2 + 1 + 2 + 2 + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_TDCS_DATA, data, sizeof(data), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}

static void nb_send_tdcs_end(uint8_t total_time)
{
  /* CMD 0x22 Brodcat EEG Operation End: Total PackageNum(4) + Reserved(1) */
  uint8_t data[2];
  data[0] = total_time;
  data[1] = 0x00;

  uint8_t out[2 + 1 + 2 + 2 + 1 + 1];
  uint16_t outn = nb_build_packet(NB_CMD_BC_TDCS_END, data, sizeof(data), out, sizeof(out));
  if (outn) nb_send_both(out, outn);
}



void oper_eeg_start( void )
{

}
void oper_eeg_stop( void )
{
	nb_send_eeg_end(g_nb_seq);
}

void oper_fnirs_start( void )
{
	g_nb_pack_fnirs = 0;
	g_nb_seq_fnirs = 0;
	afe_task_start();
	ppg_task_start(&g_ppg);
}
void oper_fnirs_stop( void )
{
	g_nb_pack_fnirs = 0;
	ppg_task_stop(&g_ppg);
	afe_task_stop();
	nb_send_fnirs_end(g_nb_seq);
}

void oper_tdcs_start( void )
{

}
void oper_tdcs_stop( void )
{
	nb_send_tdcs_end(g_nb_seq);
}

static void proc_handle_packet(nb_ch_t from, const nb_pkt_t *pkt)
{
  (void)from;

  if (pkt->cmd == NB_CMD_RES_DEVICE_BEACON) {
    return;
  }

  if (pkt->cmd == NB_CMD_REQ_OP_START) {					// 측정 ?��?�� 명령
    uint8_t sub = (pkt->len >= 1) ? pkt->data[0] : 0x01;
    (void)sub;

    if((sub&NB_SUBCMD_EEG))
	{
		// EEG STOP
    	g_nb_eeg_on = 1;
    	g_nb_dev_status = (g_nb_dev_status | 0x01);
		oper_eeg_start();

	}
	if((sub&NB_SUBCMD_FNIRS))
	{
		// fNIRS STOP
		g_nb_fnirs_on = 1;
		g_nb_dev_status = (g_nb_dev_status | (0x01<<1));
		oper_fnirs_start();

	}
	if((sub&NB_SUBCMD_TDCS))
	{
		// TDCS STOP
		g_nb_tdcs_on = 1;
		g_nb_dev_status = (g_nb_dev_status | (0x01<<2));
		oper_tdcs_start();
	}



    uint8_t resp[3];
    resp[0] = 0x06;          /* ACK */
    resp[1] = g_nb_dev_status;
    resp[2] = 0x00;

    uint8_t out[2 + 1 + 2 + 3 + 1 + 1];
    uint16_t outn = nb_build_packet(NB_CMD_RES_OP_START, resp, sizeof(resp), out, sizeof(out));
    if (outn) nb_send_both(out, outn);
    return;
  }

  if (pkt->cmd == NB_CMD_REQ_OP_STOP) {


    uint8_t resp[3];
    resp[0] = 0x06;          /* ACK */
    resp[1] = g_nb_dev_status;
    resp[2] = 0x00;

    uint8_t out[2 + 1 + 2 + 3 + 1 + 1];
    uint16_t outn = nb_build_packet(NB_CMD_RES_OP_STOP, resp, sizeof(resp), out, sizeof(out));
    if (outn) nb_send_both(out, outn);

    uint8_t sub = (pkt->len >= 1) ? pkt->data[0] : 0x01;
	(void)sub;
    if((sub&NB_SUBCMD_EEG))
    {
    	// EEG STOP
    	g_nb_eeg_on = 0;
		g_nb_dev_status = (uint8_t)(g_nb_dev_status & ~0x01);
    	oper_eeg_stop();

    }
    if((sub&NB_SUBCMD_FNIRS))
	{
		// fNIRS STOP
    	g_nb_fnirs_on = 0;
		g_nb_dev_status = (uint8_t)(g_nb_dev_status & ~(0x01<<1));
    	oper_fnirs_stop();

	}
    if((sub&NB_SUBCMD_TDCS))
	{
		// TDCS STOP
    	g_nb_tdcs_on = 0;
		g_nb_dev_status = (uint8_t)(g_nb_dev_status & ~(0x01<<2));
    	oper_tdcs_stop();
	}


    return;
  }
}




static void nb_on_packet_cb(void *user, const nb_pkt_t *pkt)
{
  nb_cb_ctx_t *ctx = (nb_cb_ctx_t*)user;
  proc_handle_packet(ctx->ch, pkt);
}

static void nb_pump_pc_uart3(void)
{
  uint8_t s[PC_RX_SZ];
  uint16_t n;

  __disable_irq();
  n = pc_wptr;
  if (n > PC_RX_SZ) n = PC_RX_SZ;
  for (uint16_t i = 0; i < n; i++) s[i] = pc_rx[i];
  pc_wptr = 0;
  __enable_irq();

  for (uint16_t i = 0; i < n; i++) {
    (void)nb_parser_feed(&g_nb_parser_pc, s[i], nb_on_packet_cb, &g_nb_ctx_pc);
  }
}

static void nb_feed_ble_bytes(const uint8_t *pay, uint16_t paylen)
{
  if (!pay || paylen == 0) return;
  for (uint16_t i = 0; i < paylen; i++) {
    (void)nb_parser_feed(&g_nb_parser_ble, pay[i], nb_on_packet_cb, &g_nb_ctx_ble);
  }
}


static bool bm64_send_le_bytes(const uint8_t *data, uint16_t len);
uint16_t delay_tmp = 300;
uint8_t ble_chunk_tmp = BLE_CHUNK_MAX;

/* NB -> BLE send request state (non-blocking) */
#define NB_BLE_TX_MAX      512
static struct {
  uint8_t  active;
  uint16_t total;
  uint16_t sent;
  uint8_t  buf[NB_BLE_TX_MAX];
} g_nb_ble_tx;

static bool bm64_send_le_bytes(const uint8_t *data, uint16_t len)
{
  /* Non-blocking request:
   * - Copy payload to internal buffer and let main loop fragment+send.
   * - Returns true if accepted, false if busy/oversize.
   */
  if (!data || len == 0) return true;
  if (len > NB_BLE_TX_MAX) {
    logf("[NB->BLE] drop oversize n=%u\r\n", len);
    return false;
  }
  if (g_nb_ble_tx.active) {
    /* already sending a NB packet */
    return false;
  }

  memcpy(g_nb_ble_tx.buf, data, len);
  g_nb_ble_tx.total = len;
  g_nb_ble_tx.sent  = 0;
  g_nb_ble_tx.active = 1;
  return true;
}

static void bm64_send_le_bytes_process(void)
{
  if (!g_nb_ble_tx.active) return;
  if (!g_ble_ready) { g_nb_ble_tx.active = 0; return; }
  if (!g_le_ch_valid) return;

  uint16_t remain = (uint16_t)(g_nb_ble_tx.total - g_nb_ble_tx.sent);
  if (remain == 0) { g_nb_ble_tx.active = 0; return; }

  uint16_t chunk = (remain > ble_chunk_tmp) ? ble_chunk_tmp : remain;

  uint8_t type;
  if (g_nb_ble_tx.total <= ble_chunk_tmp) type = 0x00; /* single */
  else if (g_nb_ble_tx.sent == 0)         type = 0x01; /* start */
  else if (remain <= ble_chunk_tmp)       type = 0x03; /* end */
  else                                    type = 0x02; /* cont */

  if (bm64_send_le_data_fragment(type, g_nb_ble_tx.total, &g_nb_ble_tx.buf[g_nb_ble_tx.sent], chunk)) {
    g_nb_ble_tx.sent = (uint16_t)(g_nb_ble_tx.sent + chunk);
    if (g_nb_ble_tx.sent >= g_nb_ble_tx.total) {
      g_nb_ble_tx.active = 0;
    }
  }
}


/* -------------------- LED -------------------- */
static void led(bool on)
{
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void TP01(bool on)
{
  HAL_GPIO_WritePin(TEST_01_GPIO_Port, TEST_01_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* -------------------- BM64 frame helpers -------------------- */
static uint8_t chksum(const uint8_t* p, uint16_t n)
{
  uint32_t s = 0;
  for (uint16_t i = 0; i < n; i++) s += p[i];
  return (uint8_t)(0 - (s & 0xFF));
}



static bool uart_send(uint8_t op, const uint8_t* prm, uint16_t plen)
{
  uint16_t len = 1 + plen;
  uint16_t i = 0;

  if (g_uart1_tx_busy) {
    return false;
  }

  g_uart1_tx_buf[i++] = WAKEUP;
  g_uart1_tx_buf[i++] = START;
  g_uart1_tx_buf[i++] = (len >> 8) & 0xFF;
  g_uart1_tx_buf[i++] = len & 0xFF;
  g_uart1_tx_buf[i++] = op;

  for (uint16_t k = 0; k < plen; k++) g_uart1_tx_buf[i++] = prm[k];

  g_uart1_tx_buf[i++] = chksum(&g_uart1_tx_buf[2], (uint16_t)(2 + 1 + plen));

  g_uart1_tx_len = i;
  g_uart1_tx_busy = 1;
  if (HAL_UART_Transmit_IT(&huart1, g_uart1_tx_buf, g_uart1_tx_len) != HAL_OK) {
    g_uart1_tx_busy = 0;
    return false;
  }
  return true;
}


/* =============================================================
 * BLE TX helpers
 * ============================================================= */

static uint16_t ring_count(void)
{
  uint16_t w = s_r_w;
  uint16_t r = s_r_r;
  if (w >= r) return (uint16_t)(w - r);
  return (uint16_t)(SAMPLE_RING_CAP - (r - w));
}

static void ring_push(const adc_sample_t* s)
{
  uint16_t next = (uint16_t)((s_r_w + 1) % SAMPLE_RING_CAP);
  if (next == s_r_r) {
    s_r_drop++;
    return;
  }
  s_ring[s_r_w] = *s;
  s_r_w = next;
}

static bool ring_pop_n(adc_sample_t* out, uint16_t n)
{
  if (ring_count() < n) return false;
  for (uint16_t i = 0; i < n; i++) {
    out[i] = s_ring[s_r_r];
    s_r_r = (uint16_t)((s_r_r + 1) % SAMPLE_RING_CAP);
  }
  return true;
}

static void pack_10ch_12bit(const uint16_t in[ADC_CH_COUNT], uint8_t out15[15])
{
  // 12bit x 10ch -> 15 bytes
  // pairs (a,b): [a11..a4][a3..a0|b11..b8][b7..b0]
  int oi = 0;
  for (int i = 0; i < ADC_CH_COUNT; i += 2) {
    uint16_t a = in[i] & 0x0FFF;
    uint16_t b = in[i+1] & 0x0FFF;
    out15[oi++] = (uint8_t)((a >> 4) & 0xFF);
    out15[oi++] = (uint8_t)(((a & 0x0F) << 4) | ((b >> 8) & 0x0F));
    out15[oi++] = (uint8_t)(b & 0xFF);
  }
}

#define BLE_CHUNK_MAX_tmp  200
static bool bm64_send_le_data_fragment(uint8_t type, uint16_t total_len,
                                      const uint8_t* payload, uint16_t payload_len);
/* Fragment TX (non-blocking)
 * - Starts UART TX(IT) and waits ACK in background (via parse() in main loop).
 * - This function must be called repeatedly until it returns true (ACK ok).
 * - It returns false while waiting/busy, and also returns false on timeout/ACK fail.
 */
static volatile uint8_t  g_ack12_seen = 0;
static volatile uint8_t  g_ack12_ok   = 0;
static volatile uint8_t  g_ack12_st   = 0;
static volatile uint32_t g_ack12_backoff_until = 0;

static bool bm64_send_le_data_fragment(uint8_t type, uint16_t total_len,
                                      const uint8_t* payload, uint16_t payload_len)
{
  static uint8_t  waiting = 0;
  static uint32_t t0 = 0;

  uint32_t now = HAL_GetTick();

  uint8_t prm[1 + 1 + 2 + 2 + BLE_CHUNK_MAX_tmp];
  uint16_t i = 0;

  if (!g_ble_ready) {
    waiting = 0;
    return false;
  }
  if (!g_le_ch_valid) {
    waiting = 0;
    return false;
  }

  if (!waiting) {
    /* Backoff when BM64 reports busy/memory full (avoid tight resend loop). */
    if (now < g_ack12_backoff_until) {
      return false;
    }
    if (payload_len > ble_chunk_tmp) payload_len = ble_chunk_tmp;

    prm[i++] = g_le_ch_index;
    prm[i++] = type;
    prm[i++] = (uint8_t)((total_len >> 8) & 0xFF);
    prm[i++] = (uint8_t)(total_len & 0xFF);
    prm[i++] = (uint8_t)((payload_len >> 8) & 0xFF);
    prm[i++] = (uint8_t)(payload_len & 0xFF);
    memcpy(&prm[i], payload, payload_len);
    i += payload_len;

    /* start UART TX (IT). if busy, retry later */
    g_ack12_seen = 0;
    g_ack12_ok   = 0;

    if (!uart_send(CMD_SEND_SPP_IAP_LE_DATA, prm, i)) {
      return false;
    }

    waiting = 1;
    t0 = now;
    return false;
  }

  /* waiting ACK */
  if (g_ack12_seen) {
    waiting = 0;
    /*
     * ACK status handling:
     * - 0x00: OK, proceed.
     * - 0x04: busy, backoff and retry same fragment.
     * - 0x05: memory full, backoff (longer) and retry same fragment.
     * - 0x01: disallow -> upper layer will stop/clear.
     */
    if (g_ack12_ok) {
      return true;
    }
    if (g_ack12_st == 0x04) {
      g_ack12_backoff_until = now + 5U;
      return false;
    }
    if (g_ack12_st == 0x05) {
      g_ack12_backoff_until = now + 20U;
      return false;
    }
    return false;
  }

  if ((now - t0) >= 200U) {
    waiting = 0;
    logf("[TX] ACK timeout for 0x12 (le_ch=%02X type=%u)\r\n", g_le_ch_index, type);
    return false;
  }

  return false; /* still waiting */
}



static void adc_read_10ch(adc_sample_t* out)
{
  // NOTE:
  // - CubeMX?  ?   ADC1?   "Scan conversion"?  ????? ?  ?  ?  ?????, 10????? 채널?   Rank 1..10?  ????? 배치?  ?   ?  ?  ?  .
  // - EOC Selection?? "EOC Seq Conv"(?  ????? ??????   ?  ?????)쪽이 ?  ?  ?  ?  ?  .
  // - ?  기서?   빠른 ?????증을 ?  ?   Polling 방식?  ????? 10????? ?  ?  ?  ?  .
  memset(out, 0, sizeof(*out));

  HAL_ADC_Start(&hadc1);
  for (int i = 0; i < ADC_CH_COUNT; i++) {
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
      out->v[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
    } else {
      out->v[i] = 0;
    }
  }
  HAL_ADC_Stop(&hadc1);
}

/* ?  ?   ?  ?  : 0x00 0xAA + Len(2) + op + payload + chk */
static bool parse(uint8_t* op, uint8_t* prm, uint16_t* plen)
{
  uint8_t s[RX_SZ];
  __disable_irq();
  memcpy(s, (void*)rx, RX_SZ);
  __enable_irq();

  for (int i = 0; i < RX_SZ - 8; i++) {

    if (!(s[i] == 0x00 && s[i+1] == 0xAA)) continue;

    uint16_t L = ((uint16_t)s[i+2] << 8) | s[i+3];
    if (L < 1 || L > (1 + BM64_MAX_PAYLOAD)) continue;

    int op_idx  = i + 4;
    int pay_idx = i + 5;
    int chk_idx = i + 4 + L;
    if (chk_idx >= RX_SZ) continue;

    uint8_t chk = s[chk_idx];

    uint32_t sum = s[i+2] + s[i+3] + s[op_idx];
    for (uint16_t k = 0; k < (L - 1); k++) sum += s[pay_idx + k];
    sum += chk;

    if ((sum & 0xFF) != 0) continue;

    *op = s[op_idx];
    *plen = L - 1;

    uint16_t copy_n = *plen;
    if (copy_n > BM64_MAX_PAYLOAD) copy_n = BM64_MAX_PAYLOAD;
    for (uint16_t k = 0; k < copy_n; k++) prm[k] = s[pay_idx + k];

    rx_clear(); // ?  공했?   ?  ????? ?  ?????
    return true;
  }
  return false;
}

//uint8_t test_value = 0;
static bool wait_ack(uint8_t cmd)
{
  uint32_t t = HAL_GetTick();
  uint8_t op, p[128];
  uint16_t n;
  uint8_t seen_any = 0;

  while (HAL_GetTick() - t < ACK_TO_MS)
  {
    if (parse(&op, p, &n))
    {
      seen_any = 1;
      logf("[BM64] ACKwait op=%02X n=%u p0=%02X p1=%02X p2=%02X\r\n",
           op, n, (n>0?p[0]:0), (n>1?p[1]:0), (n>2?p[2]:0));


      if (op == EVT_CMD_ACK)
      {
        // ?  ?   A: [cmd][status]
        if (n >= 2 && p[0] == cmd) return (p[1] == 0x00);

        // ?  ?   B: [0x00][cmd][status]
        if (n >= 3 && p[0] == 0x00 && p[1] == cmd) return (p[2] == 0x00);
      }
    }
  }

  if (!seen_any) logf("[BM64] ACKwait: no frames\r\n");
  return false;
}

static bool wait_btm_on(void)
{
  uint32_t t = HAL_GetTick();
  uint8_t op, p[128];
  uint16_t n;

  while (HAL_GetTick() - t < STATUS_TO_MS)
  {
    if (parse(&op, p, &n))
    {
      if (op == EVT_BTM_STATUS)
      {
        if (n >= 1 && p[0] == 0x02) return true;
        if (n >= 2 && p[0] == 0x00 && p[1] == 0x02) return true;
      }
    }
  }
  return false;
}

static bool wait_off_evt(void)
{
  uint32_t t = HAL_GetTick();
  uint8_t op, p[128];
  uint16_t n;

  while (HAL_GetTick() - t < OFF_EVT_TO_MS)
  {
    if (parse(&op, p, &n))
    {
      if (op == EVT_POWER_EVENT)
      {
        if (n >= 2 && p[0] == 0x01 && p[1] == 0x00) {
          logf("[BM64] OFF_EVT OK (op=32 p0=01 p1=00)\r\n");
          return true;
        } else {
          logf("[BM64] OFF_EVT op=32 payload diff (n=%u p0=%02X p1=%02X)\r\n",
               n, (n>0?p[0]:0), (n>1?p[1]:0));
        }
      }
    }
  }

  logf("[BM64] OFF_EVT TIMEOUT (op=32)\r\n");
  return false;
}

/* -------------------- MMI wrappers -------------------- */
static bool mmi(uint8_t act)
{
  uint8_t p[2] = { DATA_BASE_INDEX, act };
  rx_clear();
  if (!uart_send(CMD_MMI_ACTION, p, 2)) return false;
  return wait_ack(CMD_MMI_ACTION);
}

/* -------------------- Music Control (AVRCP trigger) -------------------- */
// Music_Control (0x04): params = [Reserved][Action]
// Action values include: 0x05 Play, 0x06 Pause, 0x07 Toggle, 0x09 Next, 0x0A Prev
// (BM83 guide; BM64 UART Command Set v2.xx is ?  ?   계열)
static bool music_ctrl(uint8_t act)
{
  uint8_t p[2] = { 0x00, act };
  rx_clear();
  if (!uart_send(CMD_MUSIC_CTRL, p, 2)) return false;
  return wait_ack(CMD_MUSIC_CTRL);
}

/* -------------------- Pairing helpers -------------------- */
static bool bm64_enter_pairing_fast(void)
{
  logf("[BM64] Enter Pairing (fast)\r\n");
  return mmi(ACT_PAIR_FAST);
}

static bool bm64_exit_pairing(void)
{
  logf("[BM64] Exit Pairing\r\n");
  return mmi(ACT_EXIT_PAIR);
}

/* ON: MFB Tpw -> BTM_Status(State=0x02) */
static bool bm64_on(void)
{
  HAL_GPIO_WritePin(BM64_MFB_PORT, BM64_MFB_PIN, GPIO_PIN_SET);
  HAL_Delay(T_PW_MS);
  HAL_GPIO_WritePin(BM64_MFB_PORT, BM64_MFB_PIN, GPIO_PIN_RESET);
  HAL_Delay(300);
  return wait_btm_on();
}

/* recover + ON */
static bool bm64_recover_and_on(void)
{
  logf("[BM64] RECOVER: reset + resync\r\n");
  BM64_ResetPulse();
  bm64_uart_resync();
  logf("[BM64] Power ON start (after recover)\r\n");
  return bm64_on();
}

/* ON with fallback */
static bool bm64_on_with_fallback(void)
{
  logf("[BM64] Power ON start\r\n");
  if (bm64_on()) return true;

  logf("[BM64] ON FAIL -> try recover once\r\n");
  return bm64_recover_and_on();
}

/* OFF try: ACK?   ?  ?  ?  ??????????, OFF_EVT ?  ?   ?   ?  ????? UNKNOWN 처리 */
typedef enum { ST_OFF, ST_ON, ST_UNKNOWN } st_t;

static st_t bm64_off_try(void)
{
  rx_clear();

  bool ok = true;
  ok &= mmi(ACT_PWR_OFF_P);
  ok &= mmi(ACT_PWR_OFF_R);

  if (!ok) {
    logf("[BM64] OFF ACK FAIL -> UNKNOWN\r\n");
    return ST_UNKNOWN;
  }

  if (wait_off_evt()) {
    logf("[BM64] OFF CONFIRMED (evt)\r\n");
    return ST_OFF;
  }

  logf("[BM64] OFF not confirmed -> UNKNOWN\r\n");
  return ST_UNKNOWN;
}

void bm64_force_on(void)
{
    // MFB HIGH ?���?
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BM64_MFB_PORT, BM64_MFB_PIN, GPIO_PIN_SET);

    // RESET 1?��
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BM64_MFB_PORT, BM64_MFB_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BM64_MFB_PORT, BM64_MFB_PIN, GPIO_PIN_SET);

    HAL_Delay(500); // �??�� ??�?
}




/* -------------------- TIM7 tick -------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    // TIM7?   1ms 주기?  ????? ??????   (기존 코드  ? ?)
    static uint16_t tick100 = 0;
    static uint16_t acc256 = 0; // 0..999 ?  ?

//    if(test_value)
//	 {
//	  test_value= 0;
//	 }
//	 else
//	 {
//	  test_value= 1;
//	 }
//	 TP01(test_value);
    // 100ms flag(기존 ?  ?????)
    if (++tick100 >= 100) {
      tick100 = 0;
      g_do_100ms = 1;
    }

    // 256Hz flag ?  ?  : 1ms tick?  ?   256/1000 비율????? ?  ?
    acc256 += ADC_FS_HZ; // +256
    if (acc256 >= 1000) {
      acc256 -= 1000;
      g_do_256hz = 1;
    }
  }
}
void key_proc( void )
{

}
void backjob_proc( void )
{
	afe_measure_poll(&g_afe_meas);
	nb_pump_pc_uart3();


	if(ppg_task_process(&g_ppg))
	{
		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][0] = g_ppg.data_ch2;
		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][1] = g_ppg.data_ch2;
		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][2] = g_ppg.data_ch2;
		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][3] = g_ppg.data_ch2;

//		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][0] = (g_nb_seq_fnirs*4)+g_nb_pack_fnirs;
//		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][1] = (g_nb_seq_fnirs*4)+g_nb_pack_fnirs*10;
//		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][2] = (g_nb_seq_fnirs*4)+g_nb_pack_fnirs*20;
//		gv_i32_fnirs_save_buf[g_nb_pack_fnirs][3] = (g_nb_seq_fnirs*4)+g_nb_pack_fnirs*30;

		g_nb_pack_fnirs++;
		if(g_nb_pack_fnirs>=FNIRS_TX_PAGE_MAX)
		{
			g_nb_pack_fnirs = 0;
			TP01(1);
#ifdef TEST_FNIRS_DUMMY
			nb_send_fnirs_dummy_frame();

#else
			nb_send_fnirs_frame();
#endif
			g_nb_seq_fnirs++;
			TP01(0);

		}
//		if(test_port_value== 0)
//		{
//			test_port_value = 1;
//		}
//		else
//		{
//			test_port_value = 0;
//		}
//		TP01(test_port_value);
	}
}

uint32_t test_tx_time = 2000;
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
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* AFE4404 init */
      afe4404_init(&g_afe, &hi2c1, AFE4404_I2C_ADDR_7BIT_DEFAULT);
//      afe4404_hw_reset();
//      afe4404_apply_regset_from_working_dump(&g_afe);
//      afe_measure_init(&g_afe_meas, &g_afe, &huart2, 100);
//      afe_measure_start(&g_afe_meas);
  /* === AFE4404 AUTO MODE + MEASURE (ADDED) ===
   * This block integrates the auto REG_READ-managed driver and 100Hz measurement loop.
   * Existing BM64 / other logic is untouched.
   */
      ppg_task_init(&g_ppg,&hi2c1,&huart2,app_ble_send,GPIOC, GPIO_PIN_6 /* AFE_RESETZ */);

  //  ppg_task_hw_init_afe(&g_ppg);



  /* EEG ADS1299 init (SPI3) */
  eeg_ads1299_init(&g_eeg, &hspi3);


	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_TIM_Base_Start_IT(&htim7);

	vref_cal = *VREFINT_CAL_ADDR;

	HAL_GPIO_WritePin(BM64_RST_PORT, BM64_RST_PIN, GPIO_PIN_SET);
	BM64_ResetPulse();

	rx_start();
	rx_clear();

	HAL_GPIO_WritePin(BM64_MFB_PORT, BM64_MFB_PIN, GPIO_PIN_RESET);
	led(false);

	afe4404_reg_dump(&g_ppg.dev, &huart2);
	afe4404_sanity_check(&g_ppg.dev, &huart2, &g_afe_rdy_cnt, 1000, 100);
	st_t st = ST_OFF;

	// 버튼 ??????  ?  ????? ?  ?????
	uint8_t btn_clicks = 0;
	uint32_t btn_t0 = 0;
	const uint32_t BTN_MULTI_MS = 600;   // 600ms ?  ?   2번이????? '?  블클?????'

  nb_parser_init(&g_nb_parser_pc);
  nb_parser_init(&g_nb_parser_ble);

  pc_rx_clear();
  pc_rx_start();

//  bm64_force_on();
//  bm64_on();
  g_nb_last_beacon_ms = HAL_GetTick();
  g_nb_last_eeg_ms = HAL_GetTick();
#ifdef TEST_FNIRS_START_START
  ppg_task_start(&g_ppg);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    eeg_ads1299_task(&g_eeg);

  while (1) {
	  backjob_proc();

    /* non-blocking NB->BLE TX progress */
    bm64_send_le_bytes_process();

    /* BLE readiness watchdog:
     * When the phone disconnects, continuing to send 0x12 can cause repeated
     * ACK timeouts and disturb other BM64 features.
     * We only auto-clear in idle state (no streaming) to avoid false clears
     * during continuous TX.
     */
    if (g_ble_ready && !g_stream_on && (!g_nb_dev_status)) {
      if ((HAL_GetTick() - g_ble_last_rx_ms) > 30000U) {
        g_ble_ready = 0;
        g_le_ch_valid = 0;
        g_tx.active = 0;
        logf("[BLE] ready timeout -> disabled\r\n");
      }
    }
    // v0.1: 10 4\88  7\88 \8b 4 \uc544\uc774\ub4e4 \uc0c1\ud0dc\uc5d0\uc11c Beacon
#ifndef TEST_BEACON_NOTUSED
    if ((HAL_GetTick() - g_nb_last_beacon_ms) >= 10000) {
      g_nb_last_beacon_ms = HAL_GetTick();
      if (g_nb_dev_status == 0x00) {
        nb_send_beacon();
      }
    }
#endif

    // v0.1: EEG ON \uc2dc dummy \ub370\uc774\ud130 \uc804\uc1a1 \uc2a4\ucf00\uc904\ub7ec(\ud14c\uc2a4\ud2b8 \uc6a9)
    if (g_nb_eeg_on && (HAL_GetTick() - g_nb_last_eeg_ms) >= test_tx_time) {
      g_nb_last_eeg_ms = HAL_GetTick();
      nb_send_eeg_dummy_frame();
//      nb_send_fnirs_dummy_frame();
      g_nb_seq++;

    }


    static uint32_t t1 = 0;
    if (HAL_GetTick() - t1 >= 1000) {
      t1 = HAL_GetTick();
      logf("[BM64] RX_CNT=%lu DROP=%lu WPTR=%u\r\n", g_bm64_rx_cnt, g_bm64_rx_drop, wptr);

    }

    // ---- 버튼 ?  ?   ?  ????? (EXTI?  ?   be=true로만 ?  ?  ?  ) ----
    if (be) {
      be = false;
      uint32_t now = HAL_GetTick();
      if (btn_clicks == 0) btn_t0 = now;
      btn_clicks++;
    }

// ---- 버튼 롱프?  ?  (측정 ?  ?  ) 처리 ----
// EXTI?   ?   ? ?  ?   ? ?  ?  ?   ? ?, 루프?  ?   "버튼?   ?  ?   ?  ?  ?  ?   ?" ?  ?  ?  ?   ?   ? ?   ? 계산
if (g_btn_latch)
{
  if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_SET) // released
  {
    uint32_t now = HAL_GetTick();
    uint32_t dur = now - g_btn_press_ms;
    g_btn_latch = 0;

    if (dur >= MEAS_HOLD_MS)
    {
      // 롱프?  ?   ?: BM64 ?  ?  /?  ?   ? ?   ? 로직 ? 충돌 �? ?
      btn_clicks = 0;
      afe_pwdn_toggle_on_longpress();
    }
  }
}


    // ---- 버튼 ?  벤트 ?  ?   ----
    // 1) ?  블클?????: ON ?  ?  ?  ?   '?  ?  ????? 모드 진입'
    // 2) ?  ??????  ?????: 기존 ?  ?   ?  ????? 로직 ?  ?????
    if (btn_clicks >= 2)
    {
      btn_clicks = 0;

      if (st == ST_ON)
      {
        // ?  블클?????: ?  ?  ?????(?  ?  커버?????) 모드 진입
        // ?  /?  ?  북에?   "MCHP_xxx"(?  ?  ?   ?  ?????) ??????   ?   ?  ?  ?????
        if (!bm64_enter_pairing_fast()) {
          logf("[BM64] Pairing enter FAIL\r\n");
        }
      }
      else
      {
        // OFF/UNKNOWN?  ?   ?  블클??????? ?  ?   ?  ??????  ?????처럼 ?  ?  (?  ?  복구 ?  ?  )
        // -> ?  ?   ?  ????? 처리????? ?  기기 ?  ?   1?   ?  ??????  ????? 취급
        btn_clicks = 1;
        btn_t0 = HAL_GetTick();
      }
    }

    // ?  ??????  ????? ?  ?  (???  ?  ?  )
    if (btn_clicks == 1 && (HAL_GetTick() - btn_t0) > BTN_MULTI_MS)
    {
      btn_clicks = 0;

      if (st == ST_OFF)
      {
        if (bm64_on_with_fallback())
        {
          st = ST_ON;
          led(true);
          logf("[BM64] Power ON OK\r\n");
        }
        else
        {
          st = ST_UNKNOWN;
          led(false);
          logf("[BM64] Power ON FAIL -> UNKNOWN\r\n");
        }
      }
      else if (st == ST_ON)
      {
        logf("[BM64] Power OFF start\r\n");
        st_t r = bm64_off_try();

        if (r == ST_OFF)
        {
          st = ST_OFF;
          led(false);
          logf("[BM64] Power OFF OK (CONFIRMED)\r\n");
        }
        else
        {
          st = ST_UNKNOWN;
          // ?  매하????? ON?  ?   ?  ?????????? ?  ?
          led(true);
          logf("[BM64] Power OFF UNCERTAIN -> UNKNOWN\r\n");
        }
      }
      else // ST_UNKNOWN
      {
        if (bm64_recover_and_on())
        {
          st = ST_ON;
          led(true);
          logf("[BM64] RECOVER+ON OK\r\n");
        }
        else
        {
          st = ST_UNKNOWN;
          led(false);
          logf("[BM64] RECOVER+ON FAIL\r\n");
        }
      }
    }

    // ---- BM64 ?  벤트 ?  ?   처리 (idle?  ?  ?  ) ----
    {
      uint8_t op;
      uint8_t p[BM64_MAX_PAYLOAD];
      uint16_t n;

      if (parse(&op, p, &n)) {
        logf("[BM64] EVT op=%02X n=%u  p0=%02X p1=%02X p2=%02X p3=%02X\r\n",
             op, n,
             (n>0?p[0]:0), (n>1?p[1]:0), (n>2?p[2]:0), (n>3?p[3]:0));

        /* ACK handling for non-blocking 0x12 TX */
        if (op == EVT_CMD_ACK) {
          /* format A: [cmd][status] */
          if (n >= 2 && p[0] == CMD_SEND_SPP_IAP_LE_DATA) {
            g_ack12_seen = 1;
            g_ack12_st   = p[1];
            g_ack12_ok   = (g_ack12_st == 0x00) ? 1 : 0;

            /* Only treat 0x01(disallow) as disconnect/stop condition.
             * 0x04(busy)/0x05(mem full) are transient; handled by backoff+retry.
             */
            if (g_ack12_st == 0x01) {
              logf("[BLE] 0x12 disallow -> stop TX (st=%02X)\r\n", g_ack12_st);
              g_ble_ready = 0;
              g_le_ch_valid = 0;
              g_stream_on = 0;
              g_tx.active = 0;
              __disable_irq();
              s_r_r = s_r_w; /* flush sample ring */
              __enable_irq();
            }
          }
          /* format B: [0x00][cmd][status] */
          else if (n >= 3 && p[0] == 0x00 && p[1] == CMD_SEND_SPP_IAP_LE_DATA) {
            g_ack12_seen = 1;
            g_ack12_st   = p[2];
            g_ack12_ok   = (g_ack12_st == 0x00) ? 1 : 0;

            if (g_ack12_st == 0x01) {
              logf("[BLE] 0x12 disallow -> stop TX (st=%02X)\r\n", g_ack12_st);
              g_ble_ready = 0;
              g_le_ch_valid = 0;
              g_stream_on = 0;
              g_tx.active = 0;
              __disable_irq();
              s_r_r = s_r_w; /* flush sample ring */
              __enable_irq();
            }
          }
        }

        // ?  ????? ?  ?   것들?? ?  기서?   처리?  ?  ????? ?  버깅?   ?  ??
        if (op == EVT_BTM_STATUS) {
          // 기존 wait_btm_on?  ?   ?  ?   ?  ?  ????? ?  ?  ?  ????? 찍기????? ?  ?   OK
          logf("[BM64] BTM_STATUS ...\r\n");
        } else if (op == EVT_POWER_EVENT) {
          logf("[BM64] POWER_EVT ...\r\n");
        } else if (op == EVT_REPORT_SPP_IAP_LE_DATA) {
          // Phone -> BM64(BLE Write) -> STM(UART event 0x22)
          // payload: ChIdx(1) Type(1) TotalLen(2) PayloadLen(2) Payload...
          if (n >= 6) {
            uint8_t ch = p[0];
            uint16_t paylen = ((uint16_t)p[4] << 8) | p[5];
            const uint8_t* pay = &p[6];

            /* Mark BLE(TDT) as ready as soon as we receive any LE data.
             * This is deliberately lightweight to avoid touching existing
             * BM64 event opcode(0x32) usage in this project.
             */
            g_ble_ready = 1;
            g_ble_last_rx_ms = HAL_GetTick();

            // v0.1  \94\84  0\88 \9e\84(STX=0x02) \9d4  0\99\ec\9d\80 payload  1\9c  \93 4\ec\964\ec\98 4  94  35\ed\865  \8c\8c\ec\84\9c\eb 1\9c  2\98\eb 6 c
            if (paylen > (uint16_t)(n - 6)) {
              paylen = (uint16_t)(n - 6);
            }
            nb_feed_ble_bytes(pay, paylen);

            /* ?   FIX: 추측(bit2) ?  ?????, ?  ?  ?????  ? ?????? 채널????? ?  ?   */
            g_le_ch_index = ch;     // 0x00?   ??????
            g_le_ch_valid = 1;

            // 간단 ?  ?   ?  로토?????
            //  - ASCII: "START" / "STOP"
            //  - 1바이?  : 0x01(start), 0x00(stop)
            if (paylen == 1) {
              if (pay[0] == 0x01) { g_stream_on = 1; logf("[BLE] START (1B)\r\n"); }
              if (pay[0] == 0x00) { g_stream_on = 0; logf("[BLE] STOP  (1B)\r\n"); }
            } else if (paylen >= 5 && memcmp(pay, "START", 5) == 0) {
              g_stream_on = 1;
              logf("[BLE] START\r\n");
              logf("[BLE] cmd paylen=%u ch=%02X le_ch=%02X valid=%u\r\n",
                   paylen, ch, g_le_ch_index, g_le_ch_valid);
            } else if (paylen >= 4 && memcmp(pay, "STOP", 4) == 0) {
              g_stream_on = 0;
              logf("[BLE] STOP\r\n");
            }

            // STOP ?   TX ?  ?   ?  ?????(즉시 멈춤)
            if (!g_stream_on) {
              g_tx.active = 0;
            }
          }
        }
      }
    }

    // ---- 256Hz ?  ?  ?????: stream_on?   ?  ????? 링버?   ?  ?   ----
    if (g_stream_on && g_do_256hz)
    {
      g_do_256hz = 0;
      adc_sample_t s;
      adc_read_10ch(&s);
      ring_push(&s);
//      if(test_value)
//                 {
//               	  test_value= 0;
//                 }
//                 else
//                 {
//               	  test_value= 1;
//                 }
//                 TP01(test_value);
      static uint16_t dbg_cnt = 0;
      dbg_cnt++;
      if (dbg_cnt >= 256) {
        dbg_cnt = 0;
        logf("[ADC] push ok, ring=%u drop=%lu v0=%u\r\n",
             ring_count(), s_r_drop, s.v[0]);
      }
    }

    // ---- ?  ?  ?   ?  ?  /버퍼????? ?  ????? ?   BLE ?  ?   ----
    // ?  ?????:
    //  - 링버?  ?   FRAME_SAMPLES ?  ?   ?  ?  ????? 1 ?  ?  ?   ?  ?
    //  - ?  ?  ?  ?? 4바이?   ?  ?   + (15바이?   * N) packed payload
    //  - BM64 0x12????? 20바이?  ?   쪼개?   ?  ?  ( ? ? ?  ?  )
    if (g_stream_on)
    {
      // (1) ?  ?  ?   ??????????
      /* ?   FIX: g_le_ch_index != 0 조건 ?  ?????, valid로만 ?  ?   */
      if (!g_tx.active && ring_count() >= FRAME_SAMPLES && g_le_ch_valid)
      {
        adc_sample_t tmp[FRAME_SAMPLES];
        if (ring_pop_n(tmp, FRAME_SAMPLES)) {
          uint16_t pos = 0;
          g_tx.buf[pos++] = (uint8_t)((g_seq >> 8) & 0xFF);
          g_tx.buf[pos++] = (uint8_t)(g_seq & 0xFF);
          g_tx.buf[pos++] = (uint8_t)FRAME_SAMPLES; // N
          g_tx.buf[pos++] = 0x01; // FLAGS/VER

          for (uint16_t i = 0; i < FRAME_SAMPLES; i++) {
            uint8_t packed15[15];
            pack_10ch_12bit(tmp[i].v, packed15);
            memcpy(&g_tx.buf[pos], packed15, sizeof(packed15));
            pos += (uint16_t)sizeof(packed15);
          }

          g_tx.total = pos;
          g_tx.sent = 0;
          g_tx.active = 1;
          logf("[TX] frame ready seq=%u total=%u le_ch=%02X ring=%u\r\n",
               g_seq, g_tx.total, g_le_ch_index, ring_count());
          g_seq++;
        }
      }

      // (2) ?  ?  ?   ?  ?  (조각)
      if (g_tx.active)
      {
        uint16_t remain = (uint16_t)(g_tx.total - g_tx.sent);
        uint16_t chunk = (remain > BLE_CHUNK_MAX) ? BLE_CHUNK_MAX : remain;

        uint8_t type;
        if (g_tx.total <= BLE_CHUNK_MAX) {
          type = 0x00; // single
        } else if (g_tx.sent == 0) {
          type = 0x01; // start
        } else if (remain <= BLE_CHUNK_MAX) {
          type = 0x03; // end
        } else {
          type = 0x02; // continue
        }

        if (bm64_send_le_data_fragment(type, g_tx.total, &g_tx.buf[g_tx.sent], chunk))
        {
          g_tx.sent = (uint16_t)(g_tx.sent + chunk);
          if (g_tx.sent >= g_tx.total) {
            g_tx.active = 0;

          }
          static uint32_t last_tx_log = 0;
          if (HAL_GetTick() - last_tx_log > 1000) {
            last_tx_log = HAL_GetTick();
            logf("[TX] progress seq=%u sent=%u/%u ring=%u\r\n",
                 g_seq, g_tx.sent, g_tx.total, ring_count());
          }


        }
        else
        {
          /* busy or waiting ACK: retry in next loop (no blocking delay) */
          static uint32_t last_fail_log = 0;
          if (HAL_GetTick() - last_fail_log > 1000) {
            last_fail_log = HAL_GetTick();
            logf("[TX] frag WAIT/BUSY type=%u total=%u sent=%u chunk=%u le_ch=%02X valid=%u\r\n",
                 type, g_tx.total, g_tx.sent, chunk, g_le_ch_index, g_le_ch_valid);
          }
        }
      }
    }
    /* ADC ?  ?  (기존 ?  ?????)
     * - ?  ?  리밍 중에?   ADC ?  ???  ????? 건드리면 채널 ?  ?  ????? 꼬일 ?   ?  ?  ?????????? 비활?  ?
     */
    if (!g_stream_on && g_do_100ms)
    {
      g_do_100ms = 0;

      uint32_t adc_in5 = 0, adc_vref = 0;
      uint32_t vdda_mV = 0, vin_mV = 0;

      HAL_ADC_Start(&hadc1);

      if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
        adc_in5 = HAL_ADC_GetValue(&hadc1);

      if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
        adc_vref = HAL_ADC_GetValue(&hadc1);

      HAL_ADC_Stop(&hadc1);

      if (adc_vref != 0)
      {
        vdda_mV = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_vref, ADC_RESOLUTION_12B);
        vin_mV  = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vdda_mV, adc_in5, ADC_RESOLUTION_12B);
        // ?  ?  ?  ????? 로그 출력
//         logf("ADC=%lu VDDA=%lumV VIN=%lumV\r\n", adc_in5, vdda_mV, vin_mV);
      }
    }

// ---- 측정 중이 ? AFE_RDY?   ?  ?   ?  ?  ?   ?  ?  ?   CSV ?  ?   ----
if (g_meas_active)
{
  // 1) ?   ? 종료 체크
  if ((HAL_GetTick() - g_meas_t0_ms) >= g_meas_duration_ms)
  {
    meas_stop();
  }
  else
  {
    // 2) pending 만큼 처리 (과도?   backlog �? ?: ?   루프?   �? ? 8개만)
    uint32_t n = g_meas_pending;
    if (n > 8) n = 8;
    while (n--)
    {
      if (g_meas_pending) g_meas_pending--;
      afe_read_and_csv_send();
    }
  }
}


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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);
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
