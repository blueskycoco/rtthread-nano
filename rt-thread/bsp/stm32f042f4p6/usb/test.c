#include <stdio.h>
#include <string.h>
#include "lusb0_usb.h"
#include "usb.h"

int main(int argc, void* argv[])
{
	void *handle;
	int i = 0, j = 0, len;
	uint32_t all_len = 0;
	int send = 0;
	uint8_t cmd[64] = {0x55, 0x12, 0x33};

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
		while(1) {
			len = hid_xfer(handle, 0x81, cmd, 64, 2);
			if (len > 0) {
				all_len += len;
				printf("all_len %d\r\n", all_len);
				for(j=0; j<len; j++)
					printf("%c", cmd[j]);
				printf("\r\n");
			}
		}
	}
	close_usb(handle, 0);

	return 0;
}
