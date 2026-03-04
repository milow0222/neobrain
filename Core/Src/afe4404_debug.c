/*
 * afe4404_debug.c
 *
 *  Created on: 2026. 1. 20.
 *      Author: milow
 */
#include "afe4404_debug.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static void ulog(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        size_t len = strnlen(buf, sizeof(buf));
        HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)len, 100);
    }
}

static bool rd(afe4404_t *dev, uint8_t reg, uint32_t *v)
{
    if (!afe4404_read_auto24(dev, reg, v)) return false;
    *v &= 0xFFFFFFu;
    return true;
}

static bool wr(afe4404_t *dev, uint8_t reg, uint32_t v)
{
    return afe4404_write_cfg24(dev, reg, v & 0xFFFFFFu);
}

static void dump_one(afe4404_t *dev, UART_HandleTypeDef *huart, uint8_t reg, const char *name)
{
    uint32_t v = 0;
    if (rd(dev, reg, &v)) {
        ulog(huart, "  0x%02X %-14s = 0x%06lX (%lu)\r\n",
             reg, name, (unsigned long)v, (unsigned long)v);
    } else {
        ulog(huart, "  0x%02X %-14s = (read FAIL)\r\n", reg, name);
    }
}

bool afe4404_reg_dump_full(afe4404_t *dev,
                           UART_HandleTypeDef *huart,
                           uint8_t start,
                           uint8_t end,
                           const char *tag)
{
    if (!dev || !huart) return false;

    if (end < start) {
        uint8_t tmp = start;
        start = end;
        end = tmp;
    }

    ulog(huart, "\r\n[AFE4404] FULL REG DUMP BEGIN (%s) 0x%02X~0x%02X\r\n",
         (tag ? tag : ""), start, end);

    // 대부분의 설정 레지스터는 REG_READ=1에서 읽어야 합니다.
    // (DATA 레지스터 0x2A~0x2F는 RAW로 그대로 읽습니다.)
    // 덤프 자체가 비교 목적이므로 REG_READ를 켠 상태로 진행합니다.
    (void)afe4404_set_regread(dev, true);
    HAL_Delay(2);

    for (uint8_t r = start; r <= end; r++) {
        uint32_t v = 0;
        bool ok;

        if (r >= 0x2A && r <= 0x2F) {
            ok = afe4404_read24_raw(dev, r, &v);
        } else {
            ok = afe4404_read24_raw(dev, r, &v);
        }

        if (ok) {
            v &= 0xFFFFFFu;
            ulog(huart, "  0x%02X : 0x%06lX (%lu)\r\n",
                 r, (unsigned long)v, (unsigned long)v);
        } else {
            ulog(huart, "  0x%02X : ---- (read FAIL)\r\n", r);
        }

        if (r == 0xFF) break; // safety (should never happen)
    }

    ulog(huart, "[AFE4404] FULL REG DUMP END (%s)\r\n\r\n", (tag ? tag : ""));
    return true;
}

bool afe4404_reg_dump(afe4404_t *dev, UART_HandleTypeDef *huart)
{
    ulog(huart, "\r\n[AFE4404] REG DUMP BEGIN\r\n");

    // ---- 핵심 제어 ----
    dump_one(dev, huart, 0x21, "TIA_CFG");
    dump_one(dev, huart, 0x22, "ILED_CFG");
    dump_one(dev, huart, 0x23, "CTRL2");

    // ---- 타이밍 엔진 핵심(대표 구간) ----
    // 사용 중인 셋업에 따라 전부 덤프해도 됩니다. 여기서는 디버깅에 유용한 범위 위주.
    ulog(huart, "[TIMING] 0x01~0x14\r\n");
    for (uint8_t r = 0x01; r <= 0x14; r++) {
        uint32_t v;
        if (rd(dev, r, &v)) {
            ulog(huart, "  0x%02X = %lu\r\n", r, (unsigned long)v);
        } else {
            ulog(huart, "  0x%02X = (read FAIL)\r\n", r);
        }
    }

    ulog(huart, "[TIMING] CONV/PRF/PDN\r\n");
    dump_one(dev, huart, 0x15, "LED2CONVST");
    dump_one(dev, huart, 0x16, "LED2CONVEND");
    dump_one(dev, huart, 0x17, "ALED2CONVST");
    dump_one(dev, huart, 0x18, "ALED2CONVEND");
    dump_one(dev, huart, 0x19, "LED1CONVST");
    dump_one(dev, huart, 0x1A, "LED1CONVEND");
    dump_one(dev, huart, 0x1D, "PRPCT");
    dump_one(dev, huart, 0x32, "PDNCYCLESTC");
    dump_one(dev, huart, 0x33, "PDNCYCLEENDC");

    // ---- 데이터 레지스터 ----
    ulog(huart, "[DATA] 0x2A~0x2F\r\n");
    dump_one(dev, huart, 0x2A, "LED2VAL");
    dump_one(dev, huart, 0x2B, "ALED2/LED3");
    dump_one(dev, huart, 0x2C, "LED1VAL");
    dump_one(dev, huart, 0x2D, "ALED1VAL");
    dump_one(dev, huart, 0x2E, "LED2-ALED2");
    dump_one(dev, huart, 0x2F, "LED1-ALED1");

    ulog(huart, "[AFE4404] REG DUMP END\r\n");
    return true;
}

static bool wait_and_measure_rdy(UART_HandleTypeDef *huart,
                                 volatile uint32_t *rdy_counter,
                                 uint32_t measure_ms,
                                 float *out_hz)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t c0 = *rdy_counter;

    while ((HAL_GetTick() - t0) < measure_ms) {
        // 바쁜 루프가 싫으면 아주 짧게 쉬어도 됨
        // HAL_Delay(1);
    }

    uint32_t t1 = HAL_GetTick();
    uint32_t c1 = *rdy_counter;

    uint32_t dt = t1 - t0;
    uint32_t dc = c1 - c0;

    if (dt == 0) return false;

    *out_hz = (float)dc * 1000.0f / (float)dt;
    ulog(huart, "[ADC_RDY] count=%lu dt=%lums => %.1f Hz\r\n",
         (unsigned long)dc, (unsigned long)dt, (double)(*out_hz));
    return true;
}

bool afe4404_sanity_check(afe4404_t *dev,
                          UART_HandleTypeDef *huart,
                          volatile uint32_t *rdy_counter,
                          uint32_t measure_ms,
                          uint32_t expected_hz)
{
    ulog(huart, "\r\n[AFE4404] SANITY CHECK BEGIN\r\n");

    // (1) I2C ACK(선택): HAL_I2C_IsDeviceReady 쓰고 싶으면 여기서 추가 가능
    // 여기서는 레지스터 read로 대체합니다.
    uint32_t v23 = 0;
    if (!rd(dev, 0x23, &v23)) {
        ulog(huart, "[FAIL] I2C read REG23 failed. 배선/주소/전원/RESET 상태 확인 필요\r\n");
        return false;
    }
    ulog(huart, "[OK] I2C read REG23=0x%06lX\r\n", (unsigned long)v23);

    // (2) REG23: 내부 OSC_ENABLE(bit9) 켜보고 read-back
    // bit9=1로 세팅 (나머지 bit는 보수적으로 0 유지)
//    uint32_t set23 = (1u << 9);
    uint32_t set23 = (v23 | (1u << 9));   // 기존값 보존 + bit9만 켬
    if (!wr(dev, 0x23, set23)) {
        ulog(huart, "[FAIL] write REG23(OSC_ENABLE) failed\r\n");
        return false;
    }
    HAL_Delay(2);

    if (!rd(dev, 0x23, &v23)) {
        ulog(huart, "[FAIL] read-back REG23 failed\r\n");
        return false;
    }
    ulog(huart, "[CHK] REG23 read-back=0x%06lX (expect bit9=1)\r\n", (unsigned long)v23);
    if (((v23 >> 9) & 1u) != 1u) {
        ulog(huart, "[WARN] OSC_ENABLE bit9 not set. 외부 CLK 모드거나 write가 안 먹은 상태일 수 있음\r\n");
        // 치명적 실패는 아니지만, 클록 모드부터 정리하는 것이 좋음
    } else {
        ulog(huart, "[OK] OSC_ENABLE=1\r\n");
    }

    // (3) ADC_RDY 주파수 측정 (EXTI 카운터 필요)
    if (rdy_counter == NULL) {
        ulog(huart, "[FAIL] rdy_counter is NULL. EXTI에서 카운터 증가 구현 필요\r\n");
        return false;
    }

    float hz = 0.0f;
    if (!wait_and_measure_rdy(huart, rdy_counter, measure_ms, &hz)) {
        ulog(huart, "[FAIL] ADC_RDY measure failed\r\n");
        return false;
    }

    // 허용 오차(±5% 기본)
    float lo = (float)expected_hz * 0.95f;
    float hi = (float)expected_hz * 1.05f;

    if (hz < lo || hz > hi) {
        ulog(huart, "[WARN] ADC_RDY Hz out of range. expect=%luHz (%.1f~%.1f)\r\n",
             (unsigned long)expected_hz, (double)lo, (double)hi);
        ulog(huart, "       타이밍(PRPCT/CLKDIV) 또는 CLK 모드(내부/외부) 확인 필요\r\n");
    } else {
        ulog(huart, "[OK] ADC_RDY frequency looks good.\r\n");
    }

    // (4) 데이터 레지스터 변화 확인: 0x2E,0x2F(LED-Ambient) 10회 읽어서 변동 체크
    // LED 전류가 0이거나 LED가 실제로 안 켜지면 변화가 작을 수 있습니다.
    int32_t min2e =  2147483647, max2e = -2147483647;
    int32_t min2f =  2147483647, max2f = -2147483647;

    for (int i = 0; i < 10; i++) {
        uint32_t r2e=0, r2f=0;
        if (rd(dev, 0x2E, &r2e) && rd(dev, 0x2F, &r2f)) {
            int32_t v2e = (int32_t)((r2e & 0x800000u) ? (r2e | 0xFF000000u) : r2e);
            int32_t v2f = (int32_t)((r2f & 0x800000u) ? (r2f | 0xFF000000u) : r2f);

            if (v2e < min2e) min2e = v2e;
            if (v2e > max2e) max2e = v2e;
            if (v2f < min2f) min2f = v2f;
            if (v2f > max2f) max2f = v2f;

            ulog(huart, "  [DATA] i=%d 2E=%ld 2F=%ld\r\n", i, (long)v2e, (long)v2f);
        } else {
            ulog(huart, "[WARN] read data reg failed at i=%d\r\n", i);
        }
        HAL_Delay(10);
    }

    ulog(huart, "[DATA] 2E range: %ld ~ %ld (span=%ld)\r\n",
         (long)min2e, (long)max2e, (long)(max2e - min2e));
    ulog(huart, "[DATA] 2F range: %ld ~ %ld (span=%ld)\r\n",
         (long)min2f, (long)max2f, (long)(max2f - min2f));

    // span이 너무 작으면 LED가 안 켜지거나, 포토다이오드 입력/광학이 막혔거나, ambient만 보일 수 있음
    if ((max2e - min2e) == 0 && (max2f - min2f) == 0) {
        ulog(huart, "[WARN] DATA not changing. ILED 설정/LED 배선/TX_SUP/PD 연결/타이밍 확인 필요\r\n");
    } else {
        ulog(huart, "[OK] DATA changes observed.\r\n");
    }

    ulog(huart, "[AFE4404] SANITY CHECK END\r\n");
    return true;
}

// afe4404_debug.c

// CONTROL0(REG=0x00) bit0: REG_READ
#define AFE_REG_CONTROL0   0x00
#define AFE_REGREAD_EN     0x000001u
#define AFE_REGREAD_DIS    0x000000u

// 테스트 대상 레지스터: 타이밍(0x01) - 확실한 R/W로 쓰는게 좋음
#define TEST_REG           0x01
#define TEST_DELTA         1u

bool afe4404_i2c_rw_selftest(afe4404_t *dev, UART_HandleTypeDef *huart)
{
    uint32_t orig = 0, rb = 0, w = 0;

    ulog(huart, "\r\n[SELFTEST2] BEGIN (REG_READ control)\r\n");

    // 1) Readout Enable ON (REG_READ=1) -> 일반 레지스터 읽기 가능 모드
    if (!afe4404_set_regread(dev, true)) {
        ulog(huart, "[SELFTEST2] FAIL: write CONTROL0(REG_READ=1)\r\n");
        return false;
    }
    HAL_Delay(2);

    // 2) TEST_REG 읽기
    if (!rd(dev, TEST_REG, &orig)) {
        ulog(huart, "[SELFTEST2] FAIL: read TEST_REG 0x%02X\r\n", TEST_REG);
        return false;
    }
    ulog(huart, "[SELFTEST2] R 0x%02X(orig)=0x%06lX\r\n", TEST_REG, (unsigned long)orig);

    // 3) Write mode (REG_READ=0) -> 설정/쓰기 모드
    if (!afe4404_set_regread(dev, false)) {
        ulog(huart, "[SELFTEST2] FAIL: write CONTROL0(REG_READ=0)\r\n");
        return false;
    }
    HAL_Delay(2);

    // 4) TEST_REG에 +1 해서 쓰기
    w = (orig + TEST_DELTA) & 0x00FFFFFFu;
    if (!wr(dev, TEST_REG, w)) {
        ulog(huart, "[SELFTEST2] FAIL: write TEST_REG 0x%02X\r\n", TEST_REG);
        return false;
    }
    ulog(huart, "[SELFTEST2] W 0x%02X=0x%06lX\r\n", TEST_REG, (unsigned long)w);
    HAL_Delay(2);

    // 5) Readout Enable ON 다시 켜고 readback
    if (!afe4404_set_regread(dev, true)) {
        ulog(huart, "[SELFTEST2] FAIL: write CONTROL0(REG_READ=1) for readback\r\n");
        return false;
    }
    HAL_Delay(2);

    if (!rd(dev, TEST_REG, &rb)) {
        ulog(huart, "[SELFTEST2] FAIL: readback TEST_REG 0x%02X\r\n", TEST_REG);
        return false;
    }
    ulog(huart, "[SELFTEST2] RB 0x%02X=0x%06lX\r\n", TEST_REG, (unsigned long)rb);

    // 6) 원복(Write mode -> orig 복원)
    if (!afe4404_set_regread(dev, false)) {
        ulog(huart, "[SELFTEST2] WARN: write CONTROL0(REG_READ=0) for restore\r\n");
    }
    HAL_Delay(2);
    (void)wr(dev, TEST_REG, orig);
    HAL_Delay(2);

    // 7) 결과 판정
    if (rb != w) {
        ulog(huart, "[SELFTEST2] FAIL (RB mismatch)\r\n");
        return false;
    }

    ulog(huart, "[SELFTEST2] PASS\r\n");
    // 5) Readout Enable ON 다시 켜고 readback
        if (!afe4404_set_regread(dev, true)) {
            ulog(huart, "[SELFTEST2] FAIL: write CONTROL0(REG_READ=1) for readback\r\n");
            return false;
        }
        HAL_Delay(2);
    return true;
}
