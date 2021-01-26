#include <stdio.h>
#include <string.h>
#include "lusb0_usb.h"
#include "usb.h"

typedef struct _int64_data {
        union {
                uint64_t m_int64;
                uint8_t m_bytes[sizeof(uint64_t)];
        };
} int64_val;
int64_val g_ts1;
int64_val g_ts2;
int main(int argc, void* argv[])
{
	void *handle;
	int i = 0, j = 0, len;
	uint32_t all_len = 0;
	uint32_t all_rcv = 0;
	int send = 0, rcv_len = 0;
	uint8_t cmd[64];
	uint8_t rsp[64];
	
	if (argc >= 2)
		send = 1;

	handle = open_usb(0);

	if (handle == NULL) {
		printf("open usb failed\r\n");
		return 0;
	}
	while(1) {
		len = hid_xfer(handle, 0x81, rsp, 64, 1000);
		if (len > 0) {
			all_len += len;
			printf("all_rcv %d\t", all_len);
			for(j=0; j<len; j++)
				printf("%02x  ", rsp[j]);
			memcpy(g_ts2.m_bytes, rsp+15, 8);
			printf(" ts delata: %04ld ", g_ts2.m_int64 - g_ts1.m_int64);
			if (g_ts2.m_int64 - g_ts1.m_int64 > 1200)
				printf(" WARRNING\r\n");
			else
				printf("\r\n");
			g_ts1.m_int64 = g_ts2.m_int64;
		} else
			printf("no data\r\n");
	}
	close_usb(handle, 0);

	return 0;
}
