#ifndef __WIFI_H
#define __WIFI_H
#include <stdint.h>
uint8_t wifi_rx(uint8_t *data);
uint8_t wifi_tx(uint8_t *data, uint8_t len);
#endif
