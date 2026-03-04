/*
 * afe4404_debug.h
 *
 *  Created on: 2026. 1. 20.
 *      Author: milow
 */

#ifndef AFE4404_DEBUG_H
#define AFE4404_DEBUG_H

#include "stm32l4xx_hal.h"
#include "afe4404.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART 로그 출력용 덤프
bool afe4404_reg_dump(afe4404_t *dev, UART_HandleTypeDef *huart);

// 전체 레지스터 덤프(부팅 직후 기본값/초기화 후 비교용)
// - start~end: 예) 0x00~0x3F
// - tag: 로그 구분용 문자열(예: "POST_RESET_DEFAULT")
bool afe4404_reg_dump_full(afe4404_t *dev,
                           UART_HandleTypeDef *huart,
                           uint8_t start,
                           uint8_t end,
                           const char *tag);

// 간단 검증(리셋/REG23/OSC/ADC_RDY Hz/데이터 변화)
// rdy_counter: EXTI에서 증가시키는 카운터 포인터(필수)
// measure_ms: 500~2000ms 권장 (예: 1000ms)
// expected_hz: 100 (100Hz 목표)
bool afe4404_sanity_check(afe4404_t *dev,
                          UART_HandleTypeDef *huart,
                          volatile uint32_t *rdy_counter,
                          uint32_t measure_ms,
                          uint32_t expected_hz);

bool afe4404_i2c_rw_selftest(afe4404_t *dev, UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif

