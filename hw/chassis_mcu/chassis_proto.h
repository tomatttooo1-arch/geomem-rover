/* Chassis command serial link, protocol v0.1 (docs/design/CHASSIS_UART_PROTOCOL_20261012.md) plus the v0.2 status
 * frame 0x82 (docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md), MCU side.
 *
 * tests/test_chassis_uart.py compiles this file on the host and checks both ends produce/accept the same bytes.
 *
 * Use:
 *   UART RX interrupt:   chassis_cmd_parser_feed(&parser, byte, &cmd) -> 1 when a valid command frame completed
 *   10 ms timer:         fill chassis_status_t (latched together with the encoder frame), then
 *                        chassis_build_status(&st, buf) and send the 24 bytes without gaps
 */
#ifndef CHASSIS_PROTO_H
#define CHASSIS_PROTO_H

#include <stdint.h>

#define CHASSIS_SYNC0 0xA5u
#define CHASSIS_SYNC1 0x5Au
#define CHASSIS_TYPE_COMMAND 0x01u
#define CHASSIS_TYPE_STATUS 0x81u
#define CHASSIS_COMMAND_LEN 11u
#define CHASSIS_STATUS_LEN 24u
#define CHASSIS_TYPE_STATUS2 0x82u
#define CHASSIS_STATUS2_LEN 52u

/* command flags */
#define CHASSIS_CMD_ENABLE 0x01u
#define CHASSIS_CMD_RECALIBRATE 0x02u
#define CHASSIS_CMD_STATUS2 0x04u     /* b2: the latest valid command asks for 0x82 instead of 0x81 */

/* status bits */
#define CHASSIS_ST_HEARTBEAT 0x01u    /* b0 sees the KV260 permit heartbeat */
#define CHASSIS_ST_MOTOR_ENABLED 0x02u /* b1 driving per command */
#define CHASSIS_ST_STOP_LATCHED 0x04u /* b2 heartbeat was lost, recovery not yet satisfied */
#define CHASSIS_ST_CMD_TIMEOUT 0x08u  /* b3 no valid command for 200 ms */
#define CHASSIS_ST_GYRO_VALID 0x10u   /* b4 */
#define CHASSIS_ST_GYRO_CALIBRATED 0x20u /* b5 */
#define CHASSIS_ST_GYRO_CALIBRATING 0x40u /* b6 motion commands not executed */
#define CHASSIS_ST_DRIVER_FAULT 0x80u /* b7 */

typedef struct {
    uint8_t seq;            /* +1 per frame, wraps */
    uint16_t mcu_ms;        /* same value as the encoder frame of this tick */
    int32_t left_count;     /* cumulative, forward = increasing */
    int32_t right_count;
    int16_t gyro_z_cdps;    /* 0.01 deg/s, bias removed, counter-clockwise seen from above = positive */
    int16_t gyro_bias_cdps; /* bias in use, 0 if not calibrated */
    uint16_t battery_mv;    /* 0 if not sampled */
    uint8_t last_cmd_seq;   /* seq of the last valid command; 0xFF (with b3) if none ever */
    uint8_t status;         /* CHASSIS_ST_* */
} chassis_status_t;

/* v0.2 status frame 0x82: raw IMU, all four encoders. Raw = straight from the IMU registers: no bias removed,
 * no dead band, no "static -> 0" gating (the MCU may keep its own processed values for its own use). */
typedef struct {
    uint8_t seq;            /* +1 per frame, wraps (one counter shared with 0x81) */
    uint32_t mcu_ms;        /* same tick as the encoder frame, 32 bit */
    int32_t enc_a;          /* TIM5, cumulative, wheel forward = increasing */
    int32_t enc_b;          /* TIM4 */
    int32_t enc_c;          /* TIM3 */
    int32_t enc_d;          /* TIM2 */
    int32_t gz_sum;         /* sum of the raw gyro z samples read in this 10 ms tick (1 LSB = 2000/32768 deg/s) */
    uint8_t gz_n;           /* successful reads summed (2 at 5 ms reads; may repeat a sample) */
    uint8_t imu_err;        /* cumulative count of failed or rejected IMU reads, wraps */
    int16_t gx_mean;        /* raw gyro x, y: mean of this tick's samples */
    int16_t gy_mean;
    int16_t ax, ay, az;     /* raw accelerometer, latest read (1 LSB = 16/32768 g) */
    int16_t yaw_raw;        /* the module's own yaw (float), round(deg / 180 * 32768), latest; comparison only */
    int16_t temp_cdeg;      /* IMU temperature, 0.01 deg C; -32768 = not available */
    int16_t gyro_bias_cdps; /* the MCU's own bias estimate, information only */
    uint16_t battery_mv;
    uint8_t last_cmd_seq;
    uint8_t status;         /* CHASSIS_ST_*, same meaning as 0x81; b4 = gz_n > 0 in this tick */
} chassis_status2_t;

typedef struct {
    uint8_t seq;
    int16_t v_mm_s;         /* forward positive */
    int16_t w_mrad_s;       /* counter-clockwise seen from above positive */
    uint8_t flags;          /* CHASSIS_CMD_*; enable clear = stop, ignore v and w */
} chassis_command_t;

typedef struct {
    uint8_t buf[CHASSIS_COMMAND_LEN];
    uint8_t n;
} chassis_cmd_parser_t;

/* CRC-16/CCITT-FALSE; "123456789" -> 0x29B1. Every frame's CRC covers all bytes before it, sync included. */
uint16_t chassis_crc16(const uint8_t *data, uint32_t len);
void chassis_build_status(const chassis_status_t *st, uint8_t out[CHASSIS_STATUS_LEN]);
void chassis_build_status2(const chassis_status2_t *st, uint8_t out[CHASSIS_STATUS2_LEN]);
void chassis_cmd_parser_init(chassis_cmd_parser_t *p);
/* Feed one received byte. Returns 1 and fills *cmd when a complete frame with a good CRC ended on this byte;
 * bad CRC / unknown type are dropped silently and the parser resynchronizes on the next 0xA5 0x5A. */
int chassis_cmd_parser_feed(chassis_cmd_parser_t *p, uint8_t byte, chassis_command_t *cmd);

#endif
