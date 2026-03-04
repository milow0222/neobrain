#ifndef PPG_TASK_H
#define PPG_TASK_H

#include "stm32l4xx_hal.h"
#include "afe4404.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// NOTE: This header is written to be drop-in friendly with existing main.c that
// might refer to 'ppg_task_hw_init_afe' and field name 'dev'.

typedef enum {
    PPG_IDLE = 0,
    PPG_RUNNING,
} ppg_state_t;

typedef struct {
    afe4404_t dev;                // <-- keep name 'dev' for compatibility

    UART_HandleTypeDef *huart_mon; // usually UART2 for monitoring
    ppg_state_t state;

    uint32_t meas_ms;              // default 2000
    uint32_t t_start_ms;

    volatile uint32_t rdy_cnt;     // filtered interrupt count
    volatile uint8_t  rdy_flag;    // set in ISR, consumed in main loop

    volatile uint32_t last_rdy_ms; // debounce filter for noisy RDY
    volatile int32_t data_ch1;		// led2-amb data
    volatile int32_t data_ch2;		// led1-amb data

} ppg_task_t;

// Variadic to tolerate "too many arguments" call sites in main.c.
// Use: ppg_task_init(&g_ppg, &hi2c1, &huart2);
bool ppg_task_init(ppg_task_t *t, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_mon, ...);

// Backward-compatible init name used by some codebases
bool ppg_task_hw_init_afe(ppg_task_t *t);

// button toggle start/stop
void ppg_task_toggle(ppg_task_t *t);
void ppg_task_start(ppg_task_t *t);
void ppg_task_stop(ppg_task_t *t);

// call from EXTI callback when ADC_RDY fires (keep ISR short!)
void ppg_task_on_rdy_isr(ppg_task_t *t);

// call in while(1)
uint8_t ppg_task_process(ppg_task_t *t);

#ifdef __cplusplus
}
#endif

#endif
