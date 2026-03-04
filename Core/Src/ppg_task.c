#include "ppg_task.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "afe4404_debug.h"
#include "usart.h"

// If your project already has log_uart2(), you can switch ulog to call it.
// For now, keep this module self-contained.
static void ulog(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    if (!huart) return;

    char buf[180];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        uint16_t len = (uint16_t)strnlen(buf, sizeof(buf));
        HAL_UART_Transmit(huart, (uint8_t*)buf, len, 50);
    }
}

bool ppg_task_hw_init_afe(ppg_task_t *t)
{
    if (!t) return false;

    // HW reset then apply working regset
    afe4404_hw_reset();
    HAL_Delay(2);

    // ===== BOOT DUMP (reset 직후 전체 레지스터 덤프) =====
    // 목적: EV 보드 세팅과 "우리 init" 차이를 찾기 위한 기준점 확보
    // ※ 덤프는 비교용이므로 0x00~0x3F 전체를 출력
    afe4404_reg_dump_full(&t->dev, t->huart_mon, 0x00, 0x3F, "POST_RESET_DEFAULT");

    afe4404_i2c_rw_selftest(&t->dev, t->huart_mon);


    if (!afe4404_apply_regset_from_working_dump(&t->dev)) {
        ulog(t->huart_mon, "[AFE] init fail\r\n");
        return false;
    }

    // ===== INIT 후 전체 덤프(옵션) =====
    // 필요 시 주석 해제해서 POST_OUR_INIT 덤프를 비교하세요.
    // afe4404_reg_dump_full(&t->dev, t->huart_mon, 0x00, 0x3F, "POST_OUR_INIT");

    ulog(t->huart_mon, "[AFE] init ok (100Hz)\r\n");
    return true;
}

bool ppg_task_init(ppg_task_t *t, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart_mon, ...)
{
    memset(t, 0, sizeof(*t));
    t->huart_mon = huart_mon;
    t->meas_ms = 120000;//20000;
    t->state = PPG_IDLE;

    // AFE address 0x58 (7-bit)
    afe4404_init(&t->dev, hi2c, 0x58);

    return ppg_task_hw_init_afe(t);
}

void ppg_task_toggle(ppg_task_t *t)
{
    if (t->state == PPG_IDLE) {
        t->rdy_cnt = 0;
        t->rdy_flag = 0;
        t->t_start_ms = HAL_GetTick();
        t->last_rdy_ms = t->t_start_ms;
        t->state = PPG_RUNNING;
        ulog(t->huart_mon, "[AFE] MEAS START %lums\r\n", (unsigned long)t->meas_ms);
    } else {
        ppg_task_stop(t);
    }
}

void ppg_task_start(ppg_task_t *t)
{
    if (1) {
        t->rdy_cnt = 0;
        t->rdy_flag = 0;
        t->t_start_ms = HAL_GetTick();
        t->last_rdy_ms = t->t_start_ms;
        t->state = PPG_RUNNING;
        ulog(t->huart_mon, "[AFE] MEAS START %lums\r\n", (unsigned long)t->meas_ms);
    }
}

void ppg_task_stop(ppg_task_t *t)
{
    if (t->state == PPG_IDLE) return;
    t->state = PPG_IDLE;
    ulog(t->huart_mon, "[AFE] MEAS STOP cnt=%lu\r\n", (unsigned long)t->rdy_cnt);
}

void ppg_task_on_rdy_isr(ppg_task_t *t)
{
    if (t->state != PPG_RUNNING) return;

    // RDY pulse is extremely narrow; noise can create extra interrupts.
    // At 100Hz, nominal period is 10ms. Reject events closer than 2ms.
    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - t->last_rdy_ms) < 2) return;
    t->last_rdy_ms = now_ms;

    t->rdy_cnt++;
    t->rdy_flag = 1;
}
uint32_t now_ms;
uint8_t ppg_task_process(ppg_task_t *t)
{
    if (t->state != PPG_RUNNING)
	{
    	return 0;
    }

    now_ms = HAL_GetTick();

    // time limit auto-stop
    if ((now_ms - t->t_start_ms) >= t->meas_ms) {
        ppg_task_stop(t);
        return 0;
    }

    if (!t->rdy_flag) return 0;
    t->rdy_flag = 0;

    // Read data regs in main loop (not in ISR)
    uint32_t r2e = 0, r2f = 0;
    if (!afe4404_read24(&t->dev, 0x2E, &r2e)) return 0;
    if (!afe4404_read24(&t->dev, 0x2F, &r2f)) return 0;

    int32_t v2e = (int32_t)((r2e & 0x800000u) ? (r2e | 0xFF000000u) : r2e);
    int32_t v2f = (int32_t)((r2f & 0x800000u) ? (r2f | 0xFF000000u) : r2f);

    t->data_ch1 = v2e;
    t->data_ch2 = v2f;			// LED1-AMB

#ifdef DEBUG_LOG_FNIRS_CVS_
    uint32_t t_rel = now_ms - t->t_start_ms;
    // CSV: time_ms, seq, led2-amb, led1-amb
    ulog(t->huart_mon, "%lu,%lu,%ld,%ld\r\n",
         (unsigned long)t_rel,
         (unsigned long)t->rdy_cnt,
         (long)v2e, (long)v2f);
#endif
    return 1;
}
