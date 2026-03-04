#ifndef __EEG_ADS1299_H
#define __EEG_ADS1299_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

/*
 * ADS1299 (EEG AFE) minimal driver skeleton
 * - SPI3 + /CS + /DRDY + START + /RESET + /PWDN + CLK_SEL
 * - This file is intentionally minimal: existing BM64/FNIRS logic remains unchanged.
 *
 * Next steps:
 * 1) Register map init (CONFIG1/2/3, CHnSET, BIAS, etc.)
 * 2) RDATAC streaming + frame packing
 * 3) 4ch @125/250Hz -> keep BLE 100Hz packetization (aggregate multiple samples per packet)
 */

typedef struct
{
  SPI_HandleTypeDef *hspi;
  volatile uint32_t drdy_count;
  volatile bool drdy_flag;

  uint8_t id_reg;     // ADS1299 ID register (address 0x00)
  bool present;
} eeg_ads1299_t;

void eeg_ads1299_init(eeg_ads1299_t *dev, SPI_HandleTypeDef *hspi);
void eeg_ads1299_on_drdy_isr(eeg_ads1299_t *dev);
void eeg_ads1299_task(eeg_ads1299_t *dev);

/* Simple low-level helpers (public for early bring-up) */
bool eeg_ads1299_read_reg(eeg_ads1299_t *dev, uint8_t addr, uint8_t *val);
bool eeg_ads1299_write_reg(eeg_ads1299_t *dev, uint8_t addr, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* __EEG_ADS1299_H */
