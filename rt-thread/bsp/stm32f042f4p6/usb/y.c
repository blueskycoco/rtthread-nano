/*
 * COPYRIGHT (C) 2012, Real-Thread Information Technology Ltd
 * All rights reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-04-14     Grissiom     initial implementation
 * 2019-12-09     Steven Liu   add YMODEM send protocol
 */
#include <unistd.h>
#include "y.h"
#include "lusb0_usb.h"
#include "usb.h"
uint8_t *send_file;
uint32_t g_file_len;
void *handle;
#ifdef YMODEM_USING_CRC_TABLE 
static const uint16_t ccitt_table[256] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
static uint16_t CRC16(unsigned char *q, int len)
{
	uint16_t crc = 0;

	while (len-- > 0)
		crc = (crc << 8) ^ ccitt_table[((crc >> 8) ^ *q++) & 0xff];
	return crc;
}
#else
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
#endif

// we could only use global varible because we could not use
// rt_device_t->user_data(it is used by the serial driver)...
static struct rym_ctx *_rym_the_ctx;

uint8_t uart_rx_buf[64];
uint8_t uart_tx_buf[64];

//static rt_err_t _rym_rx_ind(rt_device_t dev, size_t size)
//{
//    return rt_sem_release(&_rym_the_ctx->sem);
//}

/* SOH/STX + seq + payload + crc */
#define _RYM_SOH_PKG_SZ (1+2+128+2)
#define _RYM_STX_PKG_SZ (1+2+1024+2)

static enum rym_code _rym_read_code(
		struct rym_ctx *ctx,
		int timeout)
{
	/* Fast path */
	//if (rt_device_read(ctx->dev, 0, ctx->buf, 1) == 1)
	//    return (enum rym_code)(*ctx->buf);

	/* Slow path */
	do
	{
		size_t rsz;

		/* No data yet, wait for one */

		rsz = hid_xfer(handle, 0x81, uart_rx_buf, 64, 1000);
		printf("%s %d: rsz %d %x\r\n", __func__, __LINE__, rsz, uart_rx_buf[0]);
		if (rsz != 64) {
			return RYM_CODE_NONE;
		}

		ctx->buf[0] = uart_rx_buf[0];
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
	send_crc = CRC16(ctx->buf + 3, _RYM_SOH_PKG_SZ - 5);
	ctx->buf[0] = code;
	ctx->buf[1] = index;
	ctx->buf[2] = index_inv;
	ctx->buf[131] = (uint8_t)(send_crc >> 8);
	ctx->buf[132] = (uint8_t)send_crc & 0xff;

	//do
	//{
	//writelen += rt_device_write(ctx->dev, 0, ctx->buf + writelen,
	//                            _RYM_SOH_PKG_SZ - writelen);

	//}
	//while (writelen < _RYM_SOH_PKG_SZ);

	hid_xfer(handle, 0x01, ctx->buf, 64, 1000);
	hid_xfer(handle, 0x01, ctx->buf+64, 64, 1000);
	hid_xfer(handle, 0x01, ctx->buf+128, 64, 1000);
	//for (i=0; i<133; i++)
	//	printf("%x ", ctx->buf[i]);
	//printf("\r\n");
	printf("send_crc %x\r\n", send_crc);
	return 0;
}

static size_t _rym_putchar(struct rym_ctx *ctx, uint8_t code)
{
	//rt_device_write(ctx->dev, 0, &code, sizeof(code));
	uart_tx_buf[0] = code;
	hid_xfer(handle, 0x01, uart_tx_buf, 64, 1000);

	//uart_tx_set();
	return 1;
}

static size_t _rym_getchar(struct rym_ctx *ctx)
{
	uint8_t getc_ack;
	int rsz;
	//while (rt_device_read(ctx->dev, 0, &getc_ack, 1) != 1)
	//{
	//rt_sem_take(&sem, RT_WAITING_FOREVER);
	while (1) {
		rsz = hid_xfer(handle, 0x81, uart_rx_buf, 64, 1000);
		if (rsz != 64) {
			continue;
		}
		printf("%s %d: getchar %x, len %d\r\n", __func__, __LINE__,uart_rx_buf[0], rsz);
		getc_ack = uart_rx_buf[0];
		break;
	}
	//}
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
	//if (ctx->on_begin && ctx->on_begin(ctx, ctx->buf + 3, data_sz - 5) != RYM_CODE_SOH)
	//    return -RYM_ERR_CODE;
	//printf("handshake ok\r\n");
	code = RYM_CODE_SOH;
	//_rym_send_packet(ctx, code, index);
	_rym_putchar(ctx, code);
	//rt_device_set_rx_indicate(ctx->dev, _rym_rx_ind);
	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_ACK)
	{
		return -RYM_ERR_ACK;
	}

	//printf("handshake ok 1\r\n");
	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_C)
	{
		return -RYM_ERR_ACK;
	}

	ctx->stage = RYM_STAGE_ESTABLISHED;
	printf("RYM_STAGE_ESTABLISHED\r\n");
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
		//if (ctx->on_data && ctx->on_data(ctx, ctx->buf + 3, data_sz - 5) != RYM_CODE_SOH)
		//    return -RYM_ERR_CODE;
		//read sending data
		_rym_putchar(ctx, RYM_CODE_SOH);
		code = RYM_CODE_SOH;
		if (g_file_len - ofs > (data_sz-5))
			memcpy(ctx->buf+3, send_file+ofs, data_sz - 5);
		else
			memcpy(ctx->buf+3, send_file+ofs, g_file_len - ofs);
		ofs += (data_sz-5);
		if (ofs >= g_file_len)
			ctx->stage = RYM_STAGE_FINISHING;
		printf("%s %d: sending %d %d...%d %d\r\n", __func__, __LINE__,index, ofs, g_file_len, ctx->stage);
		_rym_send_packet(ctx, code, index);
		index++;
		//rt_device_set_rx_indicate(ctx->dev, _rym_rx_ind);

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
	//rt_device_set_rx_indicate(ctx->dev, _rym_rx_ind);

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
		return -RYM_ERR_ACK;
	}

	getc_ack = _rym_getchar(ctx);

	if (getc_ack != RYM_CODE_C)
	{
		return -RYM_ERR_ACK;
	}
	memset(ctx->buf+3, 0, data_sz - 5);
	//if (ctx->on_end && ctx->on_end(ctx, ctx->buf + 3, data_sz - 5) != RYM_CODE_SOH)
	//    return -RYM_ERR_CODE;

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
	printf("%s %d\r\n", __func__, __LINE__);
	hid_xfer(handle, 0x01, uart_tx_buf, 64, 1000);
	err = _rym_do_send_handshake(ctx, handshake_timeout);
	if (err != 0)
	{
		close_usb(handle, 0);
		return err;
	}

	printf("%s %d\r\n", __func__, __LINE__);
	err = _rym_do_send_trans(ctx);
	if (err != 0)
	{
		close_usb(handle, 0);
		return err;
	}
	printf("%s %d\r\n", __func__, __LINE__);

	err = _rym_do_send_fin(ctx);
	if (err != 0)
	{
		close_usb(handle, 0);
		return err;
	}
	printf("%s %d\r\n", __func__, __LINE__);

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
int main(int argc, void *argv[])
{
	struct rym_ctx ctx;
	read_file(argv[1]);
	_rym_do_send(&ctx, 10);
	free(send_file);
	return 0;
}
