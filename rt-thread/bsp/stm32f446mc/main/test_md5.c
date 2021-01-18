#include <stdio.h>
#include <stdlib.h>
#include "md5.h"

int read_file(const uint8_t *file, uint8_t **buf)
{
	FILE *fp=fopen(file,"rb");
	fseek(fp,0L,SEEK_END);
	int flen=ftell(fp);
	fseek(fp,0L,SEEK_SET);

	fread(*buf,flen,1,fp);
	fclose(fp);
	printf("read %s , %d bytes\n", file, flen);
	return flen;
}

int main(int argc, void* argv[])
{
	int i;
	MD5_CTX md5; 
	unsigned char decrypt[16] = {0};    
	unsigned char *buf = (unsigned char *)malloc(300*1024);
	int len = read_file(argv[1], &buf);

	MD5Init(&md5);
	MD5Update(&md5,(unsigned char *)buf,len);
	MD5Final(&md5,decrypt);

	for (i=0; i<16; i++)
		printf("%02x", decrypt[i]);

	free(buf);
	return 0;
}
