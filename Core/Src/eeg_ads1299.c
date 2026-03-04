#include "eeg_ads1299.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* ---- ADS1299 command bytes (common TI ADS129x family) ---- */
#define ADS1299_CMD_WAKEUP   0x02
#define ADS1299_CMD_STANDBY  0x04
#define ADS1299_CMD_RESET    0x06
#define ADS1299_CMD_START    0x08
#define ADS1299_CMD_STOP     0x0A
#define ADS1299_CMD_RDATAC   0x10
#define ADS1299_CMD_SDATAC   0x11
#define ADS1299_CMD_RDATA    0x12
#define ADS1299_CMD_RREG     0x20
#define ADS1299_CMD_WREG     0x40

/* Register address */
#define ADS1299_REG_ID       0x00

/* Timing */
#define EEG_SPI_TIMEOUT_MS   10

static void eeg_cs_low(void)  { HAL_GPIO_WritePin(SPI_CS_GPIO_Port,  SPI_CS_Pin,  GPIO_PIN_RESET); }
static void eeg_cs_high(void) { HAL_GPIO_WritePin(SPI_CS_GPIO_Port,  SPI_CS_Pin,  GPIO_PIN_SET);   }

static void eeg_start_low(void)  { HAL_GPIO_WritePin(SPI_START_GPIO_Port, SPI_START_Pin, GPIO_PIN_RESET); }
static void eeg_start_high(void) { HAL_GPIO_WritePin(SPI_START_GPIO_Port, SPI_START_Pin, GPIO_PIN_SET);   }

static void eeg_reset_low(void)  { HAL_GPIO_WritePin(EEG_RESET_GPIO_Port, EEG_RESET_Pin, GPIO_PIN_RESET); }
static void eeg_reset_high(void) { HAL_GPIO_WritePin(EEG_RESET_GPIO_Port, EEG_RESET_Pin, GPIO_PIN_SET);   }

static void eeg_pwdn_high(void) { HAL_GPIO_WritePin(EEG_PWDN_GPIO_Port, EEG_PWDN_Pin, GPIO_PIN_SET);   }
static void eeg_pwdn_low(void)  { HAL_GPIO_WritePin(EEG_PWDN_GPIO_Port, EEG_PWDN_Pin, GPIO_PIN_RESET); }

static void eeg_clk_sel_low(void)  { HAL_GPIO_WritePin(CLK_SEL_GPIO_Port, CLK_SEL_Pin, GPIO_PIN_RESET); }
static void eeg_clk_sel_high(void) { HAL_GPIO_WritePin(CLK_SEL_GPIO_Port, CLK_SEL_Pin, GPIO_PIN_SET);   }

static bool spi_txrx(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t n)
{
  if (n == 0) return true;
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, (uint8_t*)tx, rx, n, EEG_SPI_TIMEOUT_MS);
  return (st == HAL_OK);
}

static bool spi_tx(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint16_t n)
{
  if (n == 0) return true;
  HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, (uint8_t*)tx, n, EEG_SPI_TIMEOUT_MS);
  return (st == HAL_OK);
}

static bool eeg_send_cmd(eeg_ads1299_t *dev, uint8_t cmd)
{
  uint8_t tx[1] = { cmd };
  eeg_cs_low();
  bool ok = spi_tx(dev->hspi, tx, 1);
  eeg_cs_high();
  return ok;
}

bool eeg_ads1299_read_reg(eeg_ads1299_t *dev, uint8_t addr, uint8_t *val)
{
  if (!dev || !dev->hspi || !val) return false;

  uint8_t tx[3];
  uint8_t rx[3];
  tx[0] = (uint8_t)(ADS1299_CMD_RREG | (addr & 0x1F));
  tx[1] = 0x00; // read 1 register => N-1 = 0
  tx[2] = 0x00; // dummy to clock out data
  memset(rx, 0, sizeof(rx));

  eeg_cs_low();
  bool ok = spi_txrx(dev->hspi, tx, rx, 3);
  eeg_cs_high();

  if (ok) *val = rx[2];
  return ok;
}

bool eeg_ads1299_write_reg(eeg_ads1299_t *dev, uint8_t addr, uint8_t val)
{
  if (!dev || !dev->hspi) return false;

  uint8_t tx[3];
  tx[0] = (uint8_t)(ADS1299_CMD_WREG | (addr & 0x1F));
  tx[1] = 0x00; // write 1 register => N-1 = 0
  tx[2] = val;

  eeg_cs_low();
  bool ok = spi_tx(dev->hspi, tx, 3);
  eeg_cs_high();

  return ok;
}

void eeg_ads1299_on_drdy_isr(eeg_ads1299_t *dev)
{
  if (!dev) return;
  dev->drdy_count++;
  dev->drdy_flag = true;
}

static void eeg_hw_pins_safe_default(void)
{
  /* Keep device in reset/stop state until SPI is ready */
  eeg_cs_high();        // /CS idle high
  eeg_start_low();      // START low
  eeg_pwdn_high();      // /PWDN high (not power-down)
  eeg_reset_high();     // /RESET high (inactive)

  /* CLK_SEL: set to external clock by default (adjust if your EVB expects internal) */
  eeg_clk_sel_low();
}

static void eeg_hw_reset_pulse(void)
{
  /* Hardware reset pulse: drive low briefly */
  eeg_reset_low();
  HAL_Delay(2);
  eeg_reset_high();
  HAL_Delay(10);
}

void eeg_ads1299_init(eeg_ads1299_t *dev, SPI_HandleTypeDef *hspi)
{
  if (!dev) return;

  memset(dev, 0, sizeof(*dev));
  dev->hspi = hspi;
  dev->present = false;

  eeg_hw_pins_safe_default();
  HAL_Delay(5);

  /* EVB power-down control (active low on ADS1299 family) */
  eeg_pwdn_high();
  HAL_Delay(5);

  eeg_hw_reset_pulse();

  /* Put device in known SPI command mode */
  (void)eeg_send_cmd(dev, ADS1299_CMD_SDATAC);
  HAL_Delay(1);

  /* Read ID register as basic presence check */
  uint8_t id = 0;
  if (eeg_ads1299_read_reg(dev, ADS1299_REG_ID, &id))
  {
    dev->id_reg = id;
    dev->present = true;
  }

  /* Keep conversions stopped until we configure registers in next step */
  (void)eeg_send_cmd(dev, ADS1299_CMD_STOP);
  eeg_start_low();
}

void eeg_ads1299_task(eeg_ads1299_t *dev)
{
  if (!dev || !dev->present) return;

  /* For now: just clear DRDY flag.
   * Next step will read status + channel data here (RDATAC).
   */
  if (dev->drdy_flag)
  {
    dev->drdy_flag = false;
  }
}
