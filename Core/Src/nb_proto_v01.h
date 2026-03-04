#ifndef NB_PROTO_V01_H
#define NB_PROTO_V01_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NB_STX  0x55
#define NB_STX2 0xAA

// STX: 0x55 0xAA (2 bytes)
#define NB_ETX 0x03

#ifndef NB_MAX_DATA
#define NB_MAX_DATA 512
#endif

typedef struct {
    uint8_t  cmd;
    uint16_t len;                 // 0..NB_MAX_DATA
    uint8_t  data[NB_MAX_DATA];
} nb_pkt_t;

typedef enum {
    NB_PARSE_OK = 0,
    NB_PARSE_ERR_CSUM,
    NB_PARSE_ERR_LEN,
    NB_PARSE_ERR_ETX
} nb_parse_err_t;

typedef void (*nb_on_packet_fn)(void *user, const nb_pkt_t *pkt);

typedef struct {
    enum {
        S_WAIT_STX = 0,
        S_WAIT_STX2,
        S_CMD,
        S_LEN_H,
        S_LEN_L,
        S_DATA,
        S_CSUM,
        S_ETX
    } st;

    nb_pkt_t pkt;
    uint16_t idx;
    uint32_t sum;     // STX..DATA 합(Checksum 계산용)
    uint8_t  csum_rx;
} nb_parser_t;

void nb_parser_init(nb_parser_t *p);

nb_parse_err_t nb_parser_feed(nb_parser_t *p, uint8_t b, nb_on_packet_fn cb, void *user);

uint16_t nb_build_packet(uint8_t cmd, const uint8_t *data, uint16_t len,
                         uint8_t *out, uint16_t out_max);

// checksum: STX부터 DATA까지 합의 하위 1바이트를 0xFF에서 뺀 값
uint8_t nb_calc_csum(uint8_t stx, uint8_t cmd, uint16_t len, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
