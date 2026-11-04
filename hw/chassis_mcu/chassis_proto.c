/* Chassis command serial link v0.1 + v0.2 status frame, MCU-side reference codec. See chassis_proto.h. */
#include "chassis_proto.h"

#include <string.h>

uint16_t chassis_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    int b;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)(v >> 24);
}

void chassis_build_status(const chassis_status_t *st, uint8_t out[CHASSIS_STATUS_LEN])
{
    out[0] = CHASSIS_SYNC0;
    out[1] = CHASSIS_SYNC1;
    out[2] = CHASSIS_TYPE_STATUS;
    out[3] = st->seq;
    put16(&out[4], st->mcu_ms);
    put32(&out[6], (uint32_t)st->left_count);
    put32(&out[10], (uint32_t)st->right_count);
    put16(&out[14], (uint16_t)st->gyro_z_cdps);
    put16(&out[16], (uint16_t)st->gyro_bias_cdps);
    put16(&out[18], st->battery_mv);
    out[20] = st->last_cmd_seq;
    out[21] = st->status;
    put16(&out[22], chassis_crc16(out, 22));          /* bytes 0..21 (sync included), low byte first */
}

void chassis_build_status2(const chassis_status2_t *st, uint8_t out[CHASSIS_STATUS2_LEN])
{
    out[0] = CHASSIS_SYNC0;
    out[1] = CHASSIS_SYNC1;
    out[2] = CHASSIS_TYPE_STATUS2;
    out[3] = st->seq;
    put32(&out[4], st->mcu_ms);
    put32(&out[8], (uint32_t)st->enc_a);
    put32(&out[12], (uint32_t)st->enc_b);
    put32(&out[16], (uint32_t)st->enc_c);
    put32(&out[20], (uint32_t)st->enc_d);
    put32(&out[24], (uint32_t)st->gz_sum);
    out[28] = st->gz_n;
    out[29] = st->imu_err;
    put16(&out[30], (uint16_t)st->gx_mean);
    put16(&out[32], (uint16_t)st->gy_mean);
    put16(&out[34], (uint16_t)st->ax);
    put16(&out[36], (uint16_t)st->ay);
    put16(&out[38], (uint16_t)st->az);
    put16(&out[40], (uint16_t)st->yaw_raw);
    put16(&out[42], (uint16_t)st->temp_cdeg);
    put16(&out[44], (uint16_t)st->gyro_bias_cdps);
    put16(&out[46], st->battery_mv);
    out[48] = st->last_cmd_seq;
    out[49] = st->status;
    put16(&out[50], chassis_crc16(out, 50));          /* bytes 0..49 (sync included), low byte first */
}

void chassis_cmd_parser_init(chassis_cmd_parser_t *p)
{
    p->n = 0;
}

/* Drop the first buffered byte and slide to the next place a frame could start. */
static void resync(chassis_cmd_parser_t *p)
{
    uint8_t i;
    for (i = 1; i < p->n; i++)
        if (p->buf[i] == CHASSIS_SYNC0 && (i + 1 == p->n || p->buf[i + 1] == CHASSIS_SYNC1))
            break;
    memmove(p->buf, p->buf + i, (size_t)(p->n - i));
    p->n = (uint8_t)(p->n - i);
}

int chassis_cmd_parser_feed(chassis_cmd_parser_t *p, uint8_t byte, chassis_command_t *cmd)
{
    if (p->n >= CHASSIS_COMMAND_LEN)
        p->n = 0;
    p->buf[p->n++] = byte;
    for (;;) {
        if (p->n >= 1 && p->buf[0] != CHASSIS_SYNC0) { resync(p); continue; }
        if (p->n >= 2 && p->buf[1] != CHASSIS_SYNC1) { resync(p); continue; }
        if (p->n >= 3 && p->buf[2] != CHASSIS_TYPE_COMMAND) { resync(p); continue; }
        if (p->n < CHASSIS_COMMAND_LEN)
            return 0;
        if (chassis_crc16(p->buf, 9) != (uint16_t)(p->buf[9] | ((uint16_t)p->buf[10] << 8))) {  /* bytes 0..8 */
            resync(p);
            continue;
        }
        cmd->seq = p->buf[3];
        cmd->v_mm_s = (int16_t)(uint16_t)(p->buf[4] | ((uint16_t)p->buf[5] << 8));
        cmd->w_mrad_s = (int16_t)(uint16_t)(p->buf[6] | ((uint16_t)p->buf[7] << 8));
        cmd->flags = p->buf[8];
        p->n = 0;
        return 1;
    }
}
