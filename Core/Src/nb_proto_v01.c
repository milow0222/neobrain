#include "nb_proto_v01.h"
#include <string.h>

void nb_parser_init(nb_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->st = S_WAIT_STX;
}

uint8_t nb_calc_csum(uint8_t stx, uint8_t cmd, uint16_t len, const uint8_t *data)
{
    uint32_t sum = (uint32_t)stx + (uint32_t)NB_STX2 + (uint32_t)cmd + (uint32_t)((len >> 8) & 0xFF) + (uint32_t)(len & 0xFF);
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(0xFFu - (uint8_t)(sum & 0xFFu));
}

uint16_t nb_build_packet(uint8_t cmd, const uint8_t *data, uint16_t len,
                         uint8_t *out, uint16_t out_max)
{
    if (!out) return 0;
    if (len > NB_MAX_DATA) return 0;

    uint16_t need = (uint16_t)(2 + 1 + 2 + len + 1 + 1);
    if (out_max < need) return 0;

    out[0] = NB_STX;
    out[1] = NB_STX2;
    out[2] = cmd;
    out[3] = (uint8_t)((len >> 8) & 0xFF);
    out[4] = (uint8_t)(len & 0xFF);
    if (len && data) memcpy(&out[5], data, len);

    uint8_t csum = nb_calc_csum(NB_STX, cmd, len, (len ? &out[5] : NULL));
    out[5 + len] = csum;
    out[6 + len] = NB_ETX;

    return need;
}

nb_parse_err_t nb_parser_feed(nb_parser_t *p, uint8_t b, nb_on_packet_fn cb, void *user)
{
    switch (p->st) {
    case S_WAIT_STX:
        if (b == NB_STX) {
            p->sum = NB_STX;
            p->idx = 0;
            p->pkt.len = 0;
            p->st = S_WAIT_STX2;
        }
        return NB_PARSE_OK;

    case S_WAIT_STX2:
        if (b == NB_STX2) {
            p->sum += NB_STX2;
            p->st = S_CMD;
        } else if (b == NB_STX) {
            // 연속된 0x55 입력 시 재동기
            p->sum = NB_STX;
            p->st = S_WAIT_STX2;
        } else {
            p->st = S_WAIT_STX;
        }
        return NB_PARSE_OK;

    case S_CMD:
        p->pkt.cmd = b;
        p->sum += b;
        p->st = S_LEN_H;
        return NB_PARSE_OK;

    case S_LEN_H:
        p->pkt.len = ((uint16_t)b << 8);
        p->sum += b;
        p->st = S_LEN_L;
        return NB_PARSE_OK;

    case S_LEN_L:
        p->pkt.len |= b;
        p->sum += b;
        p->idx = 0;

        if (p->pkt.len > NB_MAX_DATA) {
            p->st = S_WAIT_STX;
            return NB_PARSE_ERR_LEN;
        }

        p->st = (p->pkt.len == 0) ? S_CSUM : S_DATA;
        return NB_PARSE_OK;

    case S_DATA:
        p->pkt.data[p->idx++] = b;
        p->sum += b;
        if (p->idx >= p->pkt.len) p->st = S_CSUM;
        return NB_PARSE_OK;

    case S_CSUM:
        p->csum_rx = b;
        p->st = S_ETX;
        return NB_PARSE_OK;

    case S_ETX:
        if (b != NB_ETX) {
            p->st = S_WAIT_STX;
            return NB_PARSE_ERR_ETX;
        }

        {
            uint8_t csum_calc = (uint8_t)(0xFFu - (uint8_t)(p->sum & 0xFFu));
            if (csum_calc != p->csum_rx) {
                p->st = S_WAIT_STX;
                return NB_PARSE_ERR_CSUM;
            }
        }

        if (cb) cb(user, &p->pkt);
        p->st = S_WAIT_STX;
        return NB_PARSE_OK;
    }

    p->st = S_WAIT_STX;
    return NB_PARSE_OK;
}
