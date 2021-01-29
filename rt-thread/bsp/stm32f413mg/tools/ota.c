#include <stdio.h>
#include <string.h>
#include <stdint.h>

extern int enter_ota();
extern int ota(char *path);
int main(int argc, void *argv[])
{
	if (argc != 2)
		return 0;

	enter_ota();
	ota(argv[1]);
	return 0;
}
