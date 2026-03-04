#include "afe_measure.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static void ulog(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    if (!huart) return;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}

static inline uint32_t ms_per_sample(uint32_t hz)
{
    if (hz == 0) return 10;
    return (1000u / hz);
}

void afe_measure_init(afe_measure_t *m, afe4404_t *afe, UART_HandleTypeDef *huart_log, uint32_t sample_hz)
{
    memset(m, 0, sizeof(*m));
    m->afe = afe;
    m->huart_log = huart_log;
    m->period_ms = ms_per_sample(sample_hz);
    m->last_tick_ms = HAL_GetTick();
    m->streaming = false;
}

void afe_measure_start(afe_measure_t *m)
{
    if (m) m->streaming = true;
}

void afe_measure_stop(afe_measure_t *m)
{
    if (m) m->streaming = false;
}

static bool read_all_data_regs(afe_measure_t *m)
{
    bool ok = true;
    ok &= afe4404_read_data24(m->afe, 0x2A, &m->data_2a);
    ok &= afe4404_read_data24(m->afe, 0x2B, &m->data_2b);
    ok &= afe4404_read_data24(m->afe, 0x2C, &m->data_2c);
    ok &= afe4404_read_data24(m->afe, 0x2D, &m->data_2d);
    ok &= afe4404_read_data24(m->afe, 0x2E, &m->data_2e);
    ok &= afe4404_read_data24(m->afe, 0x2F, &m->data_2f);
    return ok;
}

void afe_measure_poll(afe_measure_t *m)
{
    const uint32_t now = HAL_GetTick();
    if ((now - m->last_tick_ms) < m->period_ms) return;
    m->last_tick_ms = now;

    if (!read_all_data_regs(m)) {
        m->err_cnt++;
        if ((m->err_cnt % 50u) == 1u) {
            ulog(m->huart_log, "[AFE] DATA read FAIL (err=%lu)\r\n", (unsigned long)m->err_cnt);
        }
        return;
    }

    m->ok_cnt++;

    if (m->streaming) {
//        ulog(m->huart_log,
//             "[AFE] #%lu 2A=0x%06lX(%lu) 2B=0x%06lX(%lu) 2C=0x%06lX(%lu)\r\n"
//             "           2D=0x%06lX(%lu) 2E=0x%06lX(%lu) 2F=0x%06lX(%lu)\r\n",
//             (unsigned long)m->ok_cnt,
//             (unsigned long)m->data_2a, (unsigned long)m->data_2a,
//             (unsigned long)m->data_2b, (unsigned long)m->data_2b,
//             (unsigned long)m->data_2c, (unsigned long)m->data_2c,
//             (unsigned long)m->data_2d, (unsigned long)m->data_2d,
//             (unsigned long)m->data_2e, (unsigned long)m->data_2e,
//             (unsigned long)m->data_2f, (unsigned long)m->data_2f
//        );
    }
}
