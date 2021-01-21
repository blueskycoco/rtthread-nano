#ifndef _UTILS_H
#define _UTILS_H
#include <stdint.h>
#define TYPE_WARM_BOOT 0x12345678
rt_bool_t warm_boot();
void dump_mcu_cmd(uint16_t msg_id, uint16_t cmd_id,
		uint8_t *payload, uint16_t len);
void read_ts_64(uint8_t *ts);
void uart_tx_set();
void uart_rx_set();
void uart_rsp_out(uint8_t *cmd, uint16_t len);
void reboot();
#endif
