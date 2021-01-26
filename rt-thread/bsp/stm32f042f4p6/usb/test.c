#include <stdio.h>
#include <string.h>
#include "lusb0_usb.h"
#include "usb.h"

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
				printf("%02x\t", rsp[j]);
			printf("\r\n");
		} else
			printf("no data\r\n");
	}
	close_usb(handle, 0);

	return 0;
}
