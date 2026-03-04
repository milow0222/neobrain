#ifndef AFE4404_H
#define AFE4404_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AFE4404 I2C 24-bit register driver (with REG_READ auto management)
 *
 * Key idea (datasheet behavior):
 *  - REG_READ (CONTROL0 bit0) must be 1 to read "write registers" (configuration/timing regs).
 *  - ADC output registers 0x2A~0x2F can be read regardless of REG_READ.
 *
 * This driver provides:
 *  - raw 24-bit read/write (no REG_READ changes)
 *  - config read/write wrappers that automatically switch REG_READ only when needed
 *  - data-read wrapper for 0x2A~0x2F that never toggles REG_READ
 */

#define AFE4404_I2C_ADDR_7BIT_DEFAULT   (0x58u)

#define AFE4404_REG_CONTROL0            (0x00u)
#define AFE4404_CONTROL0_REGREAD_MASK   (0x000001u)   // bit0

#define AFE4404_REG_DATA_START          (0x2Au)
#define AFE4404_REG_DATA_END            (0x2Fu)

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  i2c_addr_7bit;     // 0x58 (7-bit)

    // CONTROL0 shadow for safe REG_READ toggling.
    // If you use other CONTROL0 bits in your project, set control0_base accordingly.
    uint32_t control0_base;     // 24-bit value with REG_READ bit cleared (bit0=0)
    bool     reg_read_cached;   // last REG_READ state we set on the device
    bool     reg_read_known;    // whether cache is valid (first call will program CONTROL0)
} afe4404_t;

// Initialize handle (no bus activity)
static inline void afe4404_init(afe4404_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    dev->hi2c = hi2c;
    dev->i2c_addr_7bit = addr_7bit;

    dev->control0_base = 0u;
    dev->reg_read_cached = false;
    dev->reg_read_known  = false;
}

/* Optional: if you use other CONTROL0 bits, set a base value here (REG_READ bit must be 0 in base). */
static inline void afe4404_set_control0_base(afe4404_t *dev, uint32_t control0_base_24)
{
    dev->control0_base = (control0_base_24 & 0x00FFFFFEu); // force bit0=0
    dev->reg_read_known = false; // re-sync on next use
}

/* ---------- Low-level RAW 24-bit register access (does NOT change REG_READ) ---------- */
bool afe4404_write24_raw(afe4404_t *dev, uint8_t reg, uint32_t val24);
bool afe4404_read24_raw (afe4404_t *dev, uint8_t reg, uint32_t *val24);

/* Backward compatible aliases (existing code may call these) */
static inline bool afe4404_write24(afe4404_t *dev, uint8_t reg, uint32_t val24)
{
    return afe4404_write24_raw(dev, reg, val24);
}
static inline bool afe4404_read24(afe4404_t *dev, uint8_t reg, uint32_t *val24)
{
    return afe4404_read24_raw(dev, reg, val24);
}

/* ---------- REG_READ auto-management ---------- */
bool afe4404_set_regread(afe4404_t *dev, bool enable);

/* Configuration/timing register access (auto REG_READ management)
 * - read_cfg: ensures REG_READ=1 (readout enable)
 * - write_cfg: ensures REG_READ=0 (write mode)
 */
bool afe4404_read_cfg24 (afe4404_t *dev, uint8_t reg, uint32_t *val24);
bool afe4404_write_cfg24(afe4404_t *dev, uint8_t reg, uint32_t val24);

/* ADC output/data register read (0x2A~0x2F), never toggles REG_READ */
bool afe4404_read_data24(afe4404_t *dev, uint8_t reg, uint32_t *val24);

/* Convenience: choose cfg/data automatically by reg number */
bool afe4404_read_auto24(afe4404_t *dev, uint8_t reg, uint32_t *val24);

/* Hardware reset (uses AFE_RESETZ_Pin and AFE_RESETZ_GPIO_Port from main.h) */
void afe4404_hw_reset(void);

/* Apply known-good regset captured from the working board (100Hz, external clock mode) */
bool afe4404_apply_regset_from_working_dump(afe4404_t *dev);

#ifdef __cplusplus
}
#endif

#endif
