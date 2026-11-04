/* Host harness for chassis_proto.c, driven by tests/test_chassis_uart.py (not firmware).
 *   selftest crc                                  -> CRC of "123456789" as 4 hex digits
 *   selftest status SEQ MS L R GZ BIAS BAT ACK ST -> the 24-byte status frame as hex
 *   selftest status2 SEQ MS A B C D GZSUM GZN ERR GX GY AX AY AZ YAW TEMP BIAS BAT ACK ST -> 52-byte 0x82 frame as hex
 *   selftest parse HEX                            -> one line "seq v w flags" per accepted command frame
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_proto.h"

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "crc") == 0) {
        printf("%04x\n", chassis_crc16((const uint8_t *)"123456789", 9));
        return 0;
    }
    if (argc == 11 && strcmp(argv[1], "status") == 0) {
        chassis_status_t st;
        uint8_t out[CHASSIS_STATUS_LEN];
        int i;
        st.seq = (uint8_t)strtol(argv[2], NULL, 0);
        st.mcu_ms = (uint16_t)strtol(argv[3], NULL, 0);
        st.left_count = (int32_t)strtoll(argv[4], NULL, 0);
        st.right_count = (int32_t)strtoll(argv[5], NULL, 0);
        st.gyro_z_cdps = (int16_t)strtol(argv[6], NULL, 0);
        st.gyro_bias_cdps = (int16_t)strtol(argv[7], NULL, 0);
        st.battery_mv = (uint16_t)strtol(argv[8], NULL, 0);
        st.last_cmd_seq = (uint8_t)strtol(argv[9], NULL, 0);
        st.status = (uint8_t)strtol(argv[10], NULL, 0);
        chassis_build_status(&st, out);
        for (i = 0; i < (int)CHASSIS_STATUS_LEN; i++)
            printf("%02x", out[i]);
        printf("\n");
        return 0;
    }
    if (argc == 22 && strcmp(argv[1], "status2") == 0) {
        chassis_status2_t st;
        uint8_t out[CHASSIS_STATUS2_LEN];
        int i;
        st.seq = (uint8_t)strtol(argv[2], NULL, 0);
        st.mcu_ms = (uint32_t)strtoul(argv[3], NULL, 0);
        st.enc_a = (int32_t)strtoll(argv[4], NULL, 0);
        st.enc_b = (int32_t)strtoll(argv[5], NULL, 0);
        st.enc_c = (int32_t)strtoll(argv[6], NULL, 0);
        st.enc_d = (int32_t)strtoll(argv[7], NULL, 0);
        st.gz_sum = (int32_t)strtoll(argv[8], NULL, 0);
        st.gz_n = (uint8_t)strtol(argv[9], NULL, 0);
        st.imu_err = (uint8_t)strtol(argv[10], NULL, 0);
        st.gx_mean = (int16_t)strtol(argv[11], NULL, 0);
        st.gy_mean = (int16_t)strtol(argv[12], NULL, 0);
        st.ax = (int16_t)strtol(argv[13], NULL, 0);
        st.ay = (int16_t)strtol(argv[14], NULL, 0);
        st.az = (int16_t)strtol(argv[15], NULL, 0);
        st.yaw_raw = (int16_t)strtol(argv[16], NULL, 0);
        st.temp_cdeg = (int16_t)strtol(argv[17], NULL, 0);
        st.gyro_bias_cdps = (int16_t)strtol(argv[18], NULL, 0);
        st.battery_mv = (uint16_t)strtol(argv[19], NULL, 0);
        st.last_cmd_seq = (uint8_t)strtol(argv[20], NULL, 0);
        st.status = (uint8_t)strtol(argv[21], NULL, 0);
        chassis_build_status2(&st, out);
        for (i = 0; i < (int)CHASSIS_STATUS2_LEN; i++)
            printf("%02x", out[i]);
        printf("\n");
        return 0;
    }
    if (argc == 3 && strcmp(argv[1], "parse") == 0) {
        chassis_cmd_parser_t p;
        chassis_command_t cmd;
        const char *h = argv[2];
        size_t i, n = strlen(h);
        chassis_cmd_parser_init(&p);
        for (i = 0; i + 1 < n; i += 2) {
            char two[3] = {h[i], h[i + 1], 0};
            if (chassis_cmd_parser_feed(&p, (uint8_t)strtol(two, NULL, 16), &cmd))
                printf("%u %d %d %u\n", cmd.seq, cmd.v_mm_s, cmd.w_mrad_s, cmd.flags);
        }
        return 0;
    }
    fprintf(stderr, "usage: see header comment\n");
    return 2;
}
