#include <unistd.h>
#include "y.h"
#include "lusb0_usb.h"
#include "usb.h"
#include "md5.h"

uint8_t *send_file;
uint32_t g_file_len;
void *handle;
static uint16_t CRC16(unsigned char *q, int len)
{
	uint16_t crc;
	char i;

	crc = 0;
	while (--len >= 0)
	{
		crc = crc ^ (int) * q++ << 8;
		i = 8;
		do
		{
			if (crc & 0x8000)
				crc = crc << 1 ^ 0x1021;
			else
				crc = crc << 1;
		}
		while (--i);
	}

	return (crc);
}

static struct rym_ctx *_rym_the_ctx;
uint8_t file_name[64];
uint8_t uart_rx_buf[64];
uint8_t uart_tx_buf[64];

/* SOH/STX + seq + payload + crc */
#define _RYM_SOH_PKG_SZ (1+2+128+2)
#define _RYM_STX_PKG_SZ (1+2+1024+2)

static enum rym_code _rym_read_code(
		struct rym_ctx *ctx,
		int timeout)
{
	/* Slow path */
	do
	{
		size_t rsz;

		/* No data yet, wait for one */

		rsz = hid_xfer(handle, 0x81, uart_rx_buf, 64, 1000);
		printf("readcode %d %x %x\r\n", rsz, 
				uart_rx_buf[0], uart_rx_buf[1]);
		if (rsz != 64) {
			return RYM_CODE_NONE;
		}

		ctx->buf[0] = uart_rx_buf[1];
		return (enum rym_code)(*ctx->buf);
	}
	while (1);
}

int _rym_send_packet(
		struct rym_ctx *ctx,
		enum rym_code code,
		uint8_t index)
{
	uint16_t send_crc;
	uint8_t index_inv = ~index;
	size_t writelen = 0;
	int i;
	uint8_t tmp[64];

	send_crc = CRC16(ctx->buf + 4, _RYM_SOH_PKG_SZ - 5);
	ctx->buf[0] = 0x01;
	ctx->buf[1] = code;
	ctx->buf[2] = index;
	ctx->buf[3] = index_inv;
	ctx->buf[132] = (uint8_t)(send_crc >> 8);
	ctx->buf[133] = (uint8_t)send_crc & 0xff;

	//for (i=0; i<135; i++)
	//	printf("%02x ", ctx->buf[i]);
	//printf("\r\n");
	hid_xfer(handle, 0x01, ctx->buf, 64, 1000);
	tmp[0] = 0x01;
	//sleep(1);	
	memcpy(tmp+1, ctx->buf+64, 63);
	hid_xfer(handle, 0x01, tmp, 64, 1000);
	
	//sleep(1);	
	memcpy(tmp+1, ctx->buf+64+63, 63);
	hid_xfer(handle, 0x01, tmp, 64, 1000);
	//sleep(1);	
	return 0;
}

static size_t _rym_putchar(struct rym_ctx *ctx, uint8_t code)
{
	uart_tx_buf[0] = 0x01;
	uart_tx_buf[1] = code;
	hid_xfer(handle, 0x01, uart_tx_buf, 64, 1000);

	return 1;
}

static size_t _rym_getchar(struct rym_ctx *ctx)
{
	uint8_t getc_ack;
	int rsz;

	while (1) {
		rsz = hid_xfer(handle, 0x81, uart_rx_buf, 64, 5000);
		if (rsz != 64) {
			printf("getchar timeout\r\n");
			continue;
		}
		printf("getchar %d %x %x\r\n", rsz,
				uart_rx_buf[0], uart_rx_buf[1]);
		getc_ack = uart_rx_buf[1];
		break;
	}
	return getc_ack;
}

int _rym_do_send_handshake(
		struct rym_ctx *ctx,
		int tm_sec)
{
	enum rym_code code;
	size_t i;
	size_t data_sz;
	uint8_t index = 0;
	uint8_t getc_ack;
	char insert_0 = '\0';
	MD5_CTX md5; 
	uint8_t decrypt[16] = {0};    

	ctx->stage = RYM_STAGE_ESTABLISHING;
	data_sz = _RYM_SOH_PKG_SZ;

	/* receive C every second */
	for (i = 0; i < tm_sec; i++)
	{
		code = _rym_read_code(ctx,
				RYM_CHD_INTV_TICK);
		if (code == RYM_CODE_C)
		{
			break;
		}
	}
	if (i == tm_sec)
	{
		return -RYM_ERR_TMO;
	}

	/* congratulations, check passed. */
	code = RYM_CODE_SOH;
	_rym_putchar(ctx, code);
	memset(ctx->buf+4, 0, data_sz-6);
	ctx->buf[4] = (g_file_len >> 24) & 0xff;
	ctx->buf[5] = (g_file_len >> 16) & 0xff;
	ctx->buf[6] = (g_file_len >>  8) & 0xff;
	ctx->buf[7] = (g_file_len >>  0) & 0xff;
	MD5Init(&md5);
	MD5Update(&md5, (uint8_t *)send_file, g_file_len);
	MD5Final(&md5, decrypt);
	for (i=0; i<16; i++)
		printf("%02x", decrypt[i]);
	printf("\r\n");
	memcpy(ctx->buf+8, decrypt, 16);
	//sprintf((char *)(ctx->buf+4), "%s%c%ld", (char *)file_name, insert_0,
	//			g_file_len);
	_rym_send_packet(ctx, code, index);
	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_ACK)
	{
		return -RYM_ERR_ACK;
	}

	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_C)
	{
		return -RYM_ERR_ACK;
	}

	ctx->stage = RYM_STAGE_ESTABLISHED;
	return 0;
}

static int _rym_do_send_trans(struct rym_ctx *ctx)
{
	ctx->stage = RYM_STAGE_TRANSMITTING;
	enum rym_code code;
	size_t data_sz;
	uint32_t index = 1;
	uint8_t getc_ack;
	uint32_t ofs = 0;
	data_sz = _RYM_SOH_PKG_SZ;

	while (1)
	{
		//read sending data
		_rym_putchar(ctx, RYM_CODE_SOH);
		code = RYM_CODE_SOH;
		if (g_file_len - ofs > (data_sz-5))
			memcpy(ctx->buf+4, send_file+ofs, data_sz - 5);
		else
			memcpy(ctx->buf+4, send_file+ofs, g_file_len - ofs);
		ofs += (data_sz-5);
		if (ofs >= g_file_len)
			ctx->stage = RYM_STAGE_FINISHING;
		printf("\033[1A");
		printf("\033[K");
		printf("%s %d: sending %d %d...%d %d\r\n", __func__, __LINE__,
				index, ofs, g_file_len, ctx->stage);
		_rym_send_packet(ctx, code, index);
		index++;

		getc_ack = _rym_getchar(ctx);

		if (getc_ack != RYM_CODE_ACK)
		{
			return -RYM_ERR_ACK;
		}

		if (ctx->stage == RYM_STAGE_FINISHING)
			break;
	}

	return 0;
}

static int _rym_do_send_fin(struct rym_ctx *ctx)
{
	enum rym_code code;
	size_t data_sz;
	uint8_t index = 0;
	uint8_t getc_ack;

	data_sz = _RYM_SOH_PKG_SZ;

	_rym_putchar(ctx, RYM_CODE_EOT);
	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_NAK)
	{
		return -RYM_ERR_ACK;
	}

	_rym_putchar(ctx, RYM_CODE_EOT);
	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_ACK)
	{
		printf("%s %d\r\n", __func__, __LINE__);
		return -RYM_ERR_ACK;
	}

	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_C)
	{
		printf("%s %d\r\n", __func__, __LINE__);
		return -RYM_ERR_ACK;
	}
	memset(ctx->buf+3, 0, data_sz - 5);

	code = RYM_CODE_SOH;

	_rym_putchar(ctx, RYM_CODE_SOH);
	_rym_send_packet(ctx, code, index);

	ctx->stage = RYM_STAGE_FINISHED;

	return 0;
}

int _rym_do_send(
		struct rym_ctx *ctx,
		int handshake_timeout)
{
	int err;

	ctx->stage = RYM_STAGE_NONE;

	handle = open_usb(0);

	//hid_xfer(handle, 0x01, uart_tx_buf, 64, 1000);
	err = _rym_do_send_handshake(ctx, handshake_timeout);
	if (err != 0)
	{
		close_usb(handle, 0);
		return err;
	}

	err = _rym_do_send_trans(ctx);
	if (err != 0)
	{
		close_usb(handle, 0);
		return err;
	}

	err = _rym_do_send_fin(ctx);
	if (err != 0)
	{
		close_usb(handle, 0);
		return err;
	}

	return err;
}
int read_file(const uint8_t *file)
{
	FILE *fp=fopen(file,"rb");
	fseek(fp,0L,SEEK_END);
	int flen=ftell(fp);
	fseek(fp,0L,SEEK_SET);

	g_file_len = flen;

	send_file = (uint8_t *)malloc(flen*sizeof(uint8_t));

	fread(send_file,flen,1,fp);
	fclose(fp);
	printf("read %s , %d bytes\n", file, flen);
	return flen;
}
int ota(char *path)
{
	struct rym_ctx ctx;
	read_file(path);
	strcpy(file_name, path);
	_rym_do_send(&ctx, 10);
	free(send_file);
	return 0;
}
