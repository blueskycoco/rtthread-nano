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
#include <rtthread.h>
#include <rthw.h>
#include "ymodem.h"
#include "flash_if.h"
#include <stdint.h>
#include <stdlib.h>
#include "mcu.h"
#include "mem_list.h"

#define PARAM_S_ADDRESS 	0x08008000
#define PARAM_E_ADDRESS		0x0800bfff
#define APP_S_ADDRESS 		0x08020000
#define APP_E_ADDRESS		0x0805ffff
#define FW_LEN_OFS		0x00
#define FW_MD5_OFS		0x04
// we could only use global varible because we could not use
// rt_device_t->user_data(it is used by the serial driver)...
extern struct rt_semaphore ota_sem;
extern rt_bool_t ota_mode;
extern void uart_tx_set();
//extern void uart_rx_set();
extern uint8_t uart_rx_buf[64];
extern uint8_t uart_tx_buf[64];
static uint8_t param[16384] = {0};
static uint32_t g_ofs = 0;
/* SOH/STX + seq + payload + crc */
#define _RYM_SOH_PKG_SZ (1+2+128+2)
#define _RYM_STX_PKG_SZ (1+2+1024+2)
ALIGN(8)
static rt_uint8_t ota_stack[2048];
struct rt_thread ota_thread;

static rt_uint16_t CRC16(unsigned char *q, int len)
{
	rt_uint16_t crc;
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

static enum rym_code _rym_read_code(
		struct rym_ctx *ctx,
		rt_tick_t timeout)
{
	uint8_t *cmd;
	uint16_t len;
	do
	{
		/* No data yet, wait for one */
		if (rt_sem_take(&ota_sem, timeout) != RT_EOK) {
			//uart_rx_set();
			rt_kprintf("%s %d: timeout \r\n", __func__, __LINE__);
			return RYM_CODE_NONE;
		}
		remove_mem(TYPE_H2D, &cmd, &len);
		if (len == 0)
			continue;

		rt_kprintf("%s %d: %x %x\r\n", __func__, __LINE__, cmd[0],
				cmd[1]);
		/* Try to read one */
		ctx->buf[0] = cmd[1];
		//uart_rx_set();
		return (enum rym_code)(*ctx->buf);
	}
	while (1);
}

/* the caller should at least alloc _RYM_STX_PKG_SZ buffer */
static rt_size_t _rym_read_data(
		struct rym_ctx *ctx,
		rt_size_t len)
{
	/* we should already have had the code */
	rt_uint8_t *buf = ctx->buf;// + 1;
	rt_size_t readlen = len;
	uint8_t *cmd1;
	uint8_t *cmd2;
	uint8_t *cmd3;
	uint16_t cmd_len = 0;
	int i, j = 0;

	while (rt_sem_take(&ota_sem, RYM_WAIT_CHR_TICK) == RT_EOK)
	{
		do {
		if (j == 0)
			remove_mem(TYPE_H2D, &cmd1, &cmd_len);
		else if (j == 1)
			remove_mem(TYPE_H2D, &cmd2, &cmd_len);
		else if (j == 2)
			remove_mem(TYPE_H2D, &cmd3, &cmd_len);
		if (cmd_len == 0)
			break;
		j++;
		//rt_kprintf("%s %d: cmd_len %d, %x %x %x\r\n", __func__, __LINE__,
		//		cmd_len, cmd[0], cmd[1], cmd[2]);
		//for (i=0; i<cmd_len; i++)
		//	rt_kprintf("%02x ", cmd[i]);
		//rt_kprintf("\r\n");
		//if (readlen >= 63) {
		//	memcpy(buf+readlen-63, cmd+1, 63);
		//	readlen -= 63;
		//} else {
		//	memcpy(buf, cmd+1, readlen);
		//	readlen = 0;
		//}i
		if (j == 3) {
		rt_memcpy(buf, cmd3+1, 63);
		rt_memcpy(buf+63, cmd2+1, 63);
		rt_memcpy(buf+126, cmd1+1, len - 126);
		readlen = 0;
		}
		//uart_rx_set();
		if (readlen == 0) {
			//rt_kprintf("%s %d: readlen %d, %x %x %x\r\n", __func__, __LINE__,
			//		readlen, buf[0], buf[1], buf[2]);
			//for (i=0; i<len; i++)
			//	rt_kprintf("%02x ", buf[i]);
			//rt_kprintf("\r\n");
			return len;
		}
		} while (cmd_len != 0);
	}

			rt_kprintf("%s %d: readlen %d\r\n", __func__, __LINE__,
					readlen);
	//uart_rx_set();
	return len;
}

static rt_size_t _rym_putchar(struct rym_ctx *ctx, rt_uint8_t code)
{
	uart_tx_buf[0] = HID_REPORT_ID;
	uart_tx_buf[1] = code;
	uart_tx_set();
	rt_sem_take(&ota_sem, RT_WAITING_FOREVER);
	return 1;
}

static rt_err_t _rym_do_handshake(
		struct rym_ctx *ctx,
		int tm_sec)
{
	enum rym_code code;
	rt_size_t i;
	uint8_t md5[16];
	rt_uint16_t recv_crc, cal_crc;
	rt_size_t data_sz = _RYM_SOH_PKG_SZ;
	rt_uint8_t fpath[32] = {0};
	int j;
	int flen;

	ctx->stage = RYM_STAGE_ESTABLISHING;
	/* send C every second, so the sender could know we are waiting for it. */
	for (i = 0; i < tm_sec; i++)
	{
		_rym_putchar(ctx, RYM_CODE_C);
		code = _rym_read_code(ctx,
				100);
		rt_kprintf("%s %d code %x\r\n", __func__, __LINE__, code);
		if (code == RYM_CODE_SOH)
		{
			data_sz = _RYM_SOH_PKG_SZ;
			break;
		}
		else if (code == RYM_CODE_STX)
		{
			data_sz = _RYM_STX_PKG_SZ;
			break;
		}
	}
	rt_kprintf("%s %d: i %d, tm_sec %d\r\n", __func__, __LINE__,
			i, tm_sec);
	if (i == tm_sec)
	{
		return -RYM_ERR_TMO;
	}

	/* receive all data */
	i = _rym_read_data(ctx, data_sz);

	if (i != (data_sz))
		return -RYM_ERR_DSZ;

	/* sanity check */
	if (ctx->buf[1] != 0 || ctx->buf[2] != 0xFF)
		return -RYM_ERR_SEQ;

	recv_crc = (rt_uint16_t)(*(ctx->buf + data_sz - 2) << 8) |
			*(ctx->buf + data_sz - 1);
	cal_crc = CRC16(ctx->buf + 3, data_sz - 5);
	if (recv_crc != cal_crc) {
		rt_kprintf("%s %d: recv_crc %x, cal_crc %x\r\n", __func__,
				__LINE__, recv_crc, cal_crc);
		return -RYM_ERR_CRC;
	}

	/* congratulations, check passed. */
#if 0
	fpath[0] = '/';
	i = 3;
	j = 1;
	while (ctx->buf[i] != 0 && i < data_sz - 6) {
		fpath[j++] = ctx->buf[i++];
	}
	flen = atoi(1 + (const char *)(ctx->buf+3) +
			rt_strnlen((const char *)(ctx->buf+3), data_sz - 6));
#else
	flen =  (ctx->buf[3] << 24) |
		(ctx->buf[4] << 16) |
		(ctx->buf[5] <<  8) |
		(ctx->buf[6] <<  0);
	rt_memcpy(md5, ctx->buf + 7, 16);
#endif

        uint32_t level = rt_hw_interrupt_disable();
	if (!FLASH_If_GetWriteProtectionStatus(PARAM_S_ADDRESS))
		FLASH_If_DisableWriteProtection(PARAM_S_ADDRESS);
	if (!FLASH_If_GetWriteProtectionStatus(APP_S_ADDRESS))
		FLASH_If_DisableWriteProtection(APP_S_ADDRESS);
	rt_memcpy(param, (const void *)PARAM_S_ADDRESS, 16384);
	rt_memcpy(param, ctx->buf+3, 20);
	if (0 != FLASH_If_Erase(PARAM_S_ADDRESS, PARAM_E_ADDRESS))
		rt_kprintf("erase param failed\r\n");
	if (0 != FLASH_If_Erase(APP_S_ADDRESS, APP_E_ADDRESS))
		rt_kprintf("erase app failed\r\n");
	if (0 != FLASH_If_Write(PARAM_S_ADDRESS, (uint32_t *)param, 4096))
		rt_kprintf("write param failed\r\n");
        rt_hw_interrupt_enable(level);
	g_ofs = 0;
	rt_kprintf("handshake ok, fw len %d, md5: \r\n", flen);
	for (i=0; i<16; i++)
		rt_kprintf("%02x", md5[i]);
	rt_kprintf("\r\n");
	return RT_EOK;
}

static rt_err_t _rym_trans_data(
		struct rym_ctx *ctx,
		rt_size_t data_sz,
		enum rym_code *code)
{
	const rt_size_t tsz = 1 + 2 + data_sz + 2;
	rt_uint16_t recv_crc;

	/* seq + data + crc */
	rt_size_t i = _rym_read_data(ctx, tsz);
	if (i != tsz)
		return -RYM_ERR_DSZ;
	rt_kprintf("%s %d\r\n", __func__, __LINE__);
	if ((ctx->buf[1] + ctx->buf[2]) != 0xFF)
	{
		return -RYM_ERR_SEQ;
	}
	rt_kprintf("%s %d\r\n", __func__, __LINE__);
	/* As we are sending C continuously, there is a chance that the
	 * sender(remote) receive an C after sending the first handshake package.
	 * So the sender will interpret it as NAK and re-send the package. So we
	 * just ignore it and proceed. */
	if (ctx->stage == RYM_STAGE_ESTABLISHED && ctx->buf[1] == 0x00)
	{
		*code = RYM_CODE_NONE;
		return RT_EOK;
	}

	rt_kprintf("%s %d\r\n", __func__, __LINE__);
	ctx->stage = RYM_STAGE_TRANSMITTING;
	/* sanity check */
	recv_crc = (rt_uint16_t)(*(ctx->buf + tsz - 2) << 8) |
			*(ctx->buf + tsz-1);
	if (recv_crc != CRC16(ctx->buf + 3, data_sz)) {
		rt_kprintf("%s %d: recv_crc %x, cal_crc %x\r\n", __func__,
				__LINE__, recv_crc, CRC16(ctx->buf + 3, data_sz));
		return -RYM_ERR_CRC;
	}
        uint32_t level = rt_hw_interrupt_disable();
	if (0 != FLASH_If_Write((uint32_t)(APP_S_ADDRESS + g_ofs),
			(uint32_t *)(ctx->buf+3), data_sz/4))
		rt_kprintf("flash write %x failed\r\n", APP_S_ADDRESS+g_ofs);
        rt_hw_interrupt_enable(level);
	g_ofs += data_sz;
	/* congratulations, check passed. */
	*code = RYM_CODE_ACK;
	return RT_EOK;
}

static rt_err_t _rym_do_trans(struct rym_ctx *ctx)
{
	rt_kprintf("%s %d\r\n", __func__, __LINE__);
	_rym_putchar(ctx, RYM_CODE_ACK);
	rt_thread_mdelay(250);
	_rym_putchar(ctx, RYM_CODE_C);
	ctx->stage = RYM_STAGE_ESTABLISHED;

	while (1)
	{
		rt_err_t err;
		enum rym_code code;
		rt_size_t data_sz, i;
		code = _rym_read_code(ctx,
				RYM_WAIT_PKG_TICK);
	rt_kprintf("%s %d, code %x\r\n", __func__, __LINE__, code);
		switch (code)
		{
			case RYM_CODE_SOH:
				data_sz = 128;
				break;
			case RYM_CODE_STX:
				data_sz = 1024;
				break;
			case RYM_CODE_EOT:
				return RT_EOK;
			default:
				return -RYM_ERR_CODE;
		};
		err = _rym_trans_data(ctx, data_sz, &code);
		if (err != RT_EOK)
			return err;
		switch (code)
		{
			case RYM_CODE_CAN:
				/* the spec require multiple CAN */
				for (i = 0; i < RYM_END_SESSION_SEND_CAN_NUM;
						i++)
				{
					_rym_putchar(ctx, RYM_CODE_CAN);
				}
				return -RYM_ERR_CAN;
			case RYM_CODE_ACK:
				_rym_putchar(ctx, RYM_CODE_ACK);
				break;
			default:
				// wrong code
				break;
		};
	}
}

static rt_err_t _rym_do_fin(struct rym_ctx *ctx)
{
	enum rym_code code;
	rt_uint16_t recv_crc;
	rt_size_t i;
	rt_size_t data_sz;

	ctx->stage = RYM_STAGE_FINISHING;
	/* we already got one EOT in the caller. invoke the callback if there is
	 * one. */
	FLASH_Lock();
	_rym_putchar(ctx, RYM_CODE_NAK);
	code = _rym_read_code(ctx, RYM_WAIT_PKG_TICK);
	if (code != RYM_CODE_EOT)
		return -RYM_ERR_CODE;

	_rym_putchar(ctx, RYM_CODE_ACK);
	rt_thread_mdelay(250);
	_rym_putchar(ctx, RYM_CODE_C);

	code = _rym_read_code(ctx, RYM_WAIT_PKG_TICK);
	if (code == RYM_CODE_SOH)
	{
		data_sz = _RYM_SOH_PKG_SZ;
	}
	else if (code == RYM_CODE_STX)
	{
		data_sz = _RYM_STX_PKG_SZ;
	}
	else
		return -RYM_ERR_CODE;
	rt_kprintf("data_sz %d\r\n", data_sz);
	i = _rym_read_data(ctx, data_sz);
	if (i != (data_sz))
		return -RYM_ERR_DSZ;

	/* sanity check
	*/
	if (ctx->buf[1] != 0 || ctx->buf[2] != 0xFF)
		return -RYM_ERR_SEQ;

	recv_crc = (rt_uint16_t)(*(ctx->buf + data_sz - 2) << 8) |
			*(ctx->buf + data_sz - 1);
	if (recv_crc != CRC16(ctx->buf + 3, data_sz - 5)){
		rt_kprintf("%s %d: recv_crc %x, cal_crc %x\r\n", __func__,
				__LINE__, recv_crc, CRC16(ctx->buf + 3, data_sz - 5));
		return -RYM_ERR_CRC;
	}

	/*next file transmission*/
	if (ctx->buf[3] != 0)
	{
		return RT_EOK;
	}

	/* congratulations, check passed. */
	ctx->stage = RYM_STAGE_FINISHED;

	/* put the last ACK */
	_rym_putchar(ctx, RYM_CODE_ACK);

	return RT_EOK;
}

static rt_err_t _rym_do_recv(
		struct rym_ctx *ctx,
		int handshake_timeout)
{
	rt_err_t err;

	ctx->stage = RYM_STAGE_NONE;

	err = _rym_do_handshake(ctx, handshake_timeout);
	if (err != RT_EOK)
	{
		return err;
	}
	while (1)
	{
		err = _rym_do_trans(ctx);
		if (err != RT_EOK)
		{
			return err;
		}

		err = _rym_do_fin(ctx);
		if (err != RT_EOK)
		{
			return err;
		}

		if (ctx->stage == RYM_STAGE_FINISHED) {
			verify_and_jump();
			break;
		}
	}
	return err;
}

static void ota_handler(void *param)
{
	struct rym_ctx ctx;
	rt_thread_mdelay(100);
	ota_mode = RT_TRUE;
	while (1)
		_rym_do_recv(&ctx, RT_WAITING_FOREVER);
}

void ota_start()
{
	FLASH_If_Init();
    	rt_thread_init(&ota_thread, "ota", ota_handler, RT_NULL,
                            ota_stack, sizeof(ota_stack), RT_THREAD_PRIORITY_MAX / 3, 20);
	rt_thread_startup(&ota_thread);
}
