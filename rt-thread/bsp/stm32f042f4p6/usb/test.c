#include <stdio.h>
#include "lusb0_usb.h"
#include "usb.h"

int main(int argc, void* argv[])
{
	void *handle;
	handle = open_usb(0);

	if (handle == NULL) {
		printf("open usb failed\r\n");
		return 0;
	}

	close_usb(handle, 0);

	return 0;
}
