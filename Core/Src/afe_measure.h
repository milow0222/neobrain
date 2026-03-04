#ifndef AFE_MEASURE_H
#define AFE_MEASURE_H

#include "afe4404.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========= Measurement loop template =========
 * IAR/legacy C mode note:
 *  - Some toolchains build C in C90 mode where 'inline' is not supported.
 *  - Therefore, start/stop are provided as normal functions (not inline).
 */

typedef struct {
    afe4404_t *afe;
    UART_HandleTypeDef *huart_log;

    // timing
    uint32_t last_tick_ms;
    uint32_t period_ms;     // e.g. 10ms for 100Hz

    // stats
    uint32_t ok_cnt;
    uint32_t err_cnt;

    // last sample (24-bit raw)
    uint32_t data_2a;
    uint32_t data_2b;
    uint32_t data_2c;
    uint32_t data_2d;
    uint32_t data_2e;
    uint32_t data_2f;

    bool streaming;         // set true to periodically print
} afe_measure_t;

void afe_measure_init(afe_measure_t *m, afe4404_t *afe, UART_HandleTypeDef *huart_log, uint32_t sample_hz);

/* Call this frequently in main loop (non-blocking pacing by HAL_GetTick). */
void afe_measure_poll(afe_measure_t *m);

/* Start/stop streaming prints (and later you can replace with BLE send) */
void afe_measure_start(afe_measure_t *m);
void afe_measure_stop (afe_measure_t *m);

#ifdef __cplusplus
}
#endif

#endif
