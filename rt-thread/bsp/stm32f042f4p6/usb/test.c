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
	if (send == 1) {
		for (i=33; i<=126; i++) {
			memset(cmd, i, 64);
			cmd[62] = '\r';
			cmd[63] = '\n';
			hid_xfer(handle, 0x01, cmd, 64, 1000);
		}
	} else {
		i = 33;
		while(1) {
			len = hid_xfer(handle, 0x81, rsp, 64, 1000);
			if (len > 0) {
				all_len += len;
				//printf("all_rcv %d\r\n", all_len);
				for(j=0; j<len; j++)
					printf("%c", rsp[j]);
				//printf(" %d\r\n", sizeof(cmd));
			}
#if 0
			memset(cmd, 0x33, 64);
			cmd[62] = '\r';
			cmd[63] = '\n';
#endif
			memset(cmd, i, 64);
			cmd[62] = '\r';
			cmd[63] = '\n';
			rcv_len = hid_xfer(handle, 0x01, cmd, 64, 1000);
			all_rcv += rcv_len;
			i++;
			if (i == 127)
				i = 33;
			//printf("all_send %d\r\n", all_rcv);
		}
	}
	close_usb(handle, 0);

	return 0;
}
