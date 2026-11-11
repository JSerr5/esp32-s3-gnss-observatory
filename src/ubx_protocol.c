/*
 * ubx_protocol.c
 * ---------------------------------------------------------------------------
 * Agente 3 - Implementacion del nucleo UBX (Fletcher-8, constructor, parser).
 * ---------------------------------------------------------------------------
 */
#include <string.h>
#include "ubx_protocol.h"
#include "ubx_defs.h"

enum {
    ST_SYNC1 = 0,
    ST_SYNC2,
    ST_CLASS,
    ST_ID,
    ST_LEN1,
    ST_LEN2,
    ST_PAYLOAD,
    ST_CKA,
    ST_CKB
};

void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

size_t ubx_build_frame(uint8_t *out, size_t cap, uint8_t cls, uint8_t id,
                       const void *payload, uint16_t len)
{
    size_t total = (size_t)len + UBX_FRAME_OVERHEAD;
    if (out == NULL || cap < total) {
        return 0;
    }
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    if (len > 0 && payload != NULL) {
        memcpy(&out[6], payload, len);
    } else if (len > 0) {
        memset(&out[6], 0, len);
    }
    uint8_t a, b;
    ubx_checksum(&out[2], (size_t)len + 4, &a, &b);
    out[6 + len] = a;
    out[7 + len] = b;
    return total;
}

void ubx_parser_init(ubx_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = ST_SYNC1;
}

static inline void ck_add(ubx_parser_t *p, uint8_t b)
{
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

ubx_feed_t ubx_parser_feed(ubx_parser_t *p, uint8_t byte)
{
    switch (p->state) {
    case ST_SYNC1:
        if (byte == UBX_SYNC1) {
            p->state = ST_SYNC2;
        }
        break;

    case ST_SYNC2:
        if (byte == UBX_SYNC2) {
            p->state = ST_CLASS;
            p->ck_a = 0;
            p->ck_b = 0;
        } else if (byte == UBX_SYNC1) {
            p->state = ST_SYNC2;          /* B5 B5 62 ... */
        } else {
            p->state = ST_SYNC1;
        }
        break;

    case ST_CLASS:
        p->cls = byte;
        ck_add(p, byte);
        p->state = ST_ID;
        break;

    case ST_ID:
        p->id = byte;
        ck_add(p, byte);
        p->state = ST_LEN1;
        break;

    case ST_LEN1:
        p->len = byte;
        ck_add(p, byte);
        p->state = ST_LEN2;
        break;

    case ST_LEN2:
        p->len = (uint16_t)(p->len | ((uint16_t)byte << 8));
        ck_add(p, byte);
        p->idx = 0;
        if (p->len > UBX_MAX_PAYLOAD) {
            p->frames_oversize++;
            p->state = ST_SYNC1;
            return UBX_FEED_OVERSIZE;
        }
        p->state = (p->len == 0) ? ST_CKA : ST_PAYLOAD;
        break;

    case ST_PAYLOAD:
        p->payload[p->idx++] = byte;
        ck_add(p, byte);
        if (p->idx >= p->len) {
            p->state = ST_CKA;
        }
        break;

    case ST_CKA:
        p->rx_ck_a = byte;
        p->state = ST_CKB;
        break;

    case ST_CKB:
        p->state = ST_SYNC1;
        if (p->rx_ck_a == p->ck_a && byte == p->ck_b) {
            p->frames_ok++;
            return UBX_FEED_FRAME;
        }
        p->frames_ckerr++;
        return UBX_FEED_CKERR;

    default:
        p->state = ST_SYNC1;
        break;
    }
    return UBX_FEED_NONE;
}

/* ----------------------------- Autoprueba ------------------------------ */
typedef struct {
    const char   *name;
    uint8_t       cls, id;
    uint8_t       payload[24];
    uint16_t      len;
    uint8_t       expect[32];
    uint8_t       expect_len;
} ubx_ref_t;

static const ubx_ref_t s_refs[] = {
    { "UBX-MON-VER poll", UBX_CLS_MON, UBX_MON_VER, {0}, 0,
      {0xB5,0x62,0x0A,0x04,0x00,0x00,0x0E,0x34}, 8 },
    { "UBX-NAV-PVT poll", UBX_CLS_NAV, UBX_NAV_PVT, {0}, 0,
      {0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19}, 8 },
    { "UBX-AID-HUI poll", UBX_CLS_AID, UBX_AID_HUI, {0}, 0,
      {0xB5,0x62,0x0B,0x02,0x00,0x00,0x0D,0x32}, 8 },
    { "UBX-CFG-RATE 5Hz", UBX_CLS_CFG, UBX_CFG_RATE,
      {0xC8,0x00,0x01,0x00,0x01,0x00}, 6,
      {0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A}, 14 },
    { "UBX-CFG-PRT 115200", UBX_CLS_CFG, UBX_CFG_PRT,
      {0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,
       0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00}, 20,
      {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,
       0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E}, 28 },
    { "UBX-CFG-CFG save BBR+EEPROM", UBX_CLS_CFG, UBX_CFG_CFG,
      {0x00,0x00,0x00,0x00,0x1F,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x05}, 13,
      {0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0x1F,0x06,0x00,0x00,
       0x00,0x00,0x00,0x00,0x05,0x46,0x05}, 21 },
};

int ubx_selftest(void (*report)(const char *name, bool ok, const uint8_t *frame, size_t len))
{
    int fails = 0;
    uint8_t buf[40];
    for (size_t i = 0; i < sizeof(s_refs) / sizeof(s_refs[0]); i++) {
        const ubx_ref_t *r = &s_refs[i];
        size_t n = ubx_build_frame(buf, sizeof(buf), r->cls, r->id,
                                   r->len ? r->payload : NULL, r->len);
        bool ok = (n == r->expect_len) && (memcmp(buf, r->expect, n) == 0);
        /* Verificacion adicional: el parser debe aceptar la trama */
        static ubx_parser_t s_tp;       /* estatico: zero-malloc, fuera de pila */
        ubx_parser_t *pp = &s_tp;
        ubx_parser_init(pp);
        ubx_feed_t res = UBX_FEED_NONE;
        for (size_t k = 0; k < n; k++) {
            res = ubx_parser_feed(pp, buf[k]);
        }
        ok = ok && (res == UBX_FEED_FRAME) && (pp->cls == r->cls) && (pp->id == r->id);
        if (!ok) {
            fails++;
        }
        if (report) {
            report(r->name, ok, buf, n);
        }
    }
    return fails;
}
