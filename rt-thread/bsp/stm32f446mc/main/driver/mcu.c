#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "mcu.h"
#include "mcu_cmd.h"
#include "rc4.h"
#include "crc.h"
#include "utils.h"
#include "mem_list.h"

static struct rt_event event_d2h;
static uint16_t sw_major_version = 0x0000;
static uint16_t sw_minor_version = 0x0001;
static uint16_t sw_patch_version = 0x0000;
static uint32_t sw_build_date = 2020082115;
static uint16_t hw_major_version = 0x0000;
static uint16_t hw_minor_version = 0x0001;
static uint16_t hw_patch_version = 0x0000;
static uint32_t hw_build_date = 2020082514;
static uint8_t heart_ts[2][8];
ALIGN(8)
static rt_uint8_t mcu_stack[2048];
struct rt_thread mcu_thread;

static void parse_host_cmd(uint8_t *cmd, uint16_t len, uint8_t *msg)
{
	uint32_t crc;
	//uint8_t msg[32] = {0};
	uint16_t msg_id, payload_len;
	uint16_t _len;

	/* protocl 
	 * |plain           |Security                 |  |plain   |
	 * 0    1..8 9 10   11 12 13 14  15 16  17...N   N+1 .. N+4
	 * STX  TS   MSGID  RESERVED     LEN    PAYLOAD  CRC
	 */

	if (cmd[0] != HOST_CMD_STX) {
		rt_kprintf("invalid cmd %x\r\n", cmd[0]);
		msg[0] = MCU_ERR_UNSUPPORT;
		msg[1] = HOST_CMD;
		goto FAIL;
	}

	if (len < 4) {
		rt_kprintf("invalid len %d\r\n", len);
		msg[0] = MCU_ERR_INV_ARG;
		msg[1] = HOST_CMD;
		goto FAIL;
	}
	_len = (cmd[len-2] << 8) | cmd[len-1];

	crc =   cmd[_len-4] |
		(cmd[_len-3] << 8) |
		(cmd[_len-2] << 16) |
		(cmd[_len-1] << 24);
	if (crc != crc32((uint8_t *)cmd, _len-4)) {
		rt_kprintf("invalid crc h %x != c %x\r\n", crc,
				crc32((uint8_t *)cmd, _len-4));
		msg[0] = MCU_ERR_CRC;
		msg[1] = HOST_CMD;
		goto FAIL;
	}

	/* msg structure 
	   0   1   2 3     4 5          6 7 8 9  10..31
	   err h/d msg_id  payload_len  reserve  payload
	   */

	msg_id = (cmd[CMD_MSGID_1] << 8) | cmd[CMD_MSGID_0];
	if (msg_id == HEART_CMD)
		read_ts_64(heart_ts[1]);

	set_rc4_key(cmd[7]%10, msg_id, cmd+CMD_TS_OFS);
	rc4(cmd+CMD_RESERVE_OFS,
			cmd+CMD_RESERVE_OFS, _len - 4 - 1 - 8 - 2);

	if (msg_id == HEART_CMD) {
		rt_memcpy(heart_ts[0], cmd+18, 8);
	}

	msg[0] = MCU_ERR_SUCCESS;
	msg[1] = HOST_CMD;
	msg[2] = cmd[CMD_MSGID_1];
	msg[3] = cmd[CMD_MSGID_0];
	msg[4] = cmd[CMD_PAYLOAD_LEN_1];
	msg[5] = cmd[CMD_PAYLOAD_LEN_0];
	rt_memcpy(msg+6, cmd+CMD_RESERVE_OFS, 4);
	payload_len = (msg[4] << 8) | msg[5];


	if (payload_len <= (64 - 10)) {
		rt_memcpy(msg+10, cmd+CMD_PAYLOAD_OFS, payload_len);
	} else {
		msg[0] = MCU_ERR_NREAL;
		msg[1] = HOST_CMD;
	}
	return;
FAIL:	
	rt_kprintf("not enough msg to send\r\n");
}

static uint16_t get_rsp_msg_id(uint16_t msg_id)
{
	switch (msg_id)
	{
		case HOST_CMD_GET:
			return HOST_CMD_GET_RSP;
		case HOST_CMD_SET:
			return HOST_CMD_SET_RSP;
		default:
			return msg_id;
	}
	return 0;
}
static void get_sw_ver()
{
	char *p = (char *)FW_VERSION;
	char *d = __DATE__;
	char *t = __TIME__;
	char major[2] = {0}, minor[3] = {0}, patch[4] = {0};
	char year[5] = {0};
	uint8_t mon = 0;
	char day[3] = {0};
	major[0] = p[3];
	major[1] = 0;
	minor[0] = p[5];
	minor[1] = p[6];
	minor[2] = 0;
	patch[0] = p[8];
	patch[1] = p[9];
	patch[2] = p[10];
	patch[3] = 0;
	sw_major_version = atoi(major);
	sw_minor_version = atoi(minor);
	sw_patch_version = atoi(patch);
	rt_memcpy(year, d+7, 4);
	rt_memcpy(day, d+4, 2);
	if (rt_strstr(d, "Jan"))
		mon = 1;
	else if (rt_strstr(d, "Feb"))
		mon = 2;
	else if (rt_strstr(d, "Mar"))
		mon = 3;
	else if (rt_strstr(d, "Apr"))
		mon = 4;
	else if (rt_strstr(d, "May"))
		mon = 5;
	else if (rt_strstr(d, "Jun"))
		mon = 6;
	else if (rt_strstr(d, "Jul"))
		mon = 7;
	else if (rt_strstr(d, "Aug"))
		mon = 8;
	else if (rt_strstr(d, "Sep"))
		mon = 9;
	else if (rt_strstr(d, "Oct"))
		mon = 10;
	else if (rt_strstr(d, "Nov"))
		mon = 11;
	else if (rt_strstr(d, "Dec"))
		mon = 12;

	t[2] = 0;
	sw_build_date = atoi(year)*1000000 + mon*10000 + atoi(day)*100 + atoi(t);
}
static uint16_t fill_payload(uint16_t msg_id, uint8_t *cmd, uint16_t cmd_len,
		uint8_t *rsp)
{
	uint16_t payload_len = 0;
	/* fill reserved */
	rsp[0] = 0x00;
	rsp[1] = 0x00;
	rsp[2] = 0x00;
	rsp[3] = 0x00;
	switch (msg_id) {
		case MSG_ID_HW_VER:
		case MSG_ID_SW_VER:
			rsp[6] = MCU_ERR_SUCCESS; 
			if (msg_id == MSG_ID_SW_VER) {
				get_sw_ver();
				rsp[8] = (sw_major_version >> 8) & 0xff;
				rsp[7] = sw_major_version & 0xff;
				rsp[10] = (sw_minor_version >> 8) & 0xff;
				rsp[9] = sw_minor_version & 0xff;
				rsp[12] = (sw_patch_version >> 8) & 0xff;
				rsp[11] = sw_patch_version & 0xff;
				rsp[16] = (sw_build_date >> 24) & 0xff;
				rsp[15] = (sw_build_date >> 16) & 0xff;
				rsp[14] = (sw_build_date >> 8) & 0xff;
				rsp[13] = (sw_build_date >> 0) & 0xff;
			} else {
				rsp[8] = (hw_major_version >> 8) & 0xff;
				rsp[7] = hw_major_version & 0xff;
				rsp[10] = (hw_minor_version >> 8) & 0xff;
				rsp[9] = hw_minor_version & 0xff;
				rsp[12] = (hw_patch_version >> 8) & 0xff;
				rsp[11] = hw_patch_version & 0xff;
				rsp[16] = (hw_build_date >> 24) & 0xff;
				rsp[15] = (hw_build_date >> 16) & 0xff;
				rsp[14] = (hw_build_date >> 8) & 0xff;
				rsp[13] = (hw_build_date >> 0) & 0xff;
			}
			payload_len = 11;
			break;
		default:
			break;

	}
	rsp[5] = (payload_len >> 8) & 0xff;
	rsp[4] = payload_len & 0xff;

	return payload_len;
}

static void mcu_msg_handler(uint8_t *msg)
{
	/* msg structure 
	   0   1   2 3     4 5          6 7 8 9  10..31
	   err h/d msg_id  payload_len  reserve  payload
	   */

	//rt_bool_t status = RT_TRUE;    
	uint8_t rsp[64] = {0};
	uint16_t rsp_msg_id = 0;
	uint16_t out_payload_len = 0;
	uint8_t err = msg[0];
	uint16_t cmd_id = 0;
	uint16_t msg_id = (msg[2] << 8) | msg[3];
	uint16_t cmd_len = (msg[4] << 8) | msg[5];
	uint8_t *cmd = (uint8_t *)(msg+10);
	//uint32_t reserve = (msg[6] << 24) | (msg[7] << 16) |
	//	(msg[8] << 8) | msg[9];

	if (msg_id == HOST_CMD_GET || msg_id == HOST_CMD_SET)
		cmd_id = (cmd[2] << 8) | cmd[1];

	if (err == MCU_ERR_SUCCESS)
		dump_mcu_cmd(msg_id, cmd_id, cmd, cmd_len);
	else {
		rt_kprintf("Error cmd %x from host\r\n", err);
		return;
	}

	/* msg 0 STX */
	rsp[0] = HOST_CMD_STX;
	/* msg 1 ts */
	read_ts_64(rsp+1);
	/* msg 9 msg_id */
	if (err == MCU_ERR_SUCCESS) {
		if (HOST_CMD == msg[1])
			rsp_msg_id = get_rsp_msg_id(msg_id);
		else
			rsp_msg_id = msg_id;
	} else {
		rsp_msg_id = 0xffff;
	}
	rsp[CMD_MSGID_1] = (rsp_msg_id >> 8) & 0xff;
	rsp[CMD_MSGID_0] = rsp_msg_id & 0xff;
	/* msg 11 reserve,payload len,payload */
	if (err == MCU_ERR_SUCCESS) {
		out_payload_len = fill_payload(msg_id, cmd, cmd_len,
				rsp+CMD_RESERVE_OFS);
	} else {
		rsp[CMD_PAYLOAD_LEN_0] = 0x01;
		rsp[CMD_PAYLOAD_LEN_1] = 0x00;
		rsp[CMD_PAYLOAD_OFS] = err;
		out_payload_len = 1;

	}
	/*int i;
	  for (i=0; i<17+out_payload_len; i++)
	  rt_kprintf("-> %x\r\n",
	  rsp[i]);*/
	/* encrypt reserve, payload len, payload */
	set_rc4_key(rsp[7]%10, rsp_msg_id, rsp+1);
	rc4(rsp+CMD_RESERVE_OFS,
			rsp+CMD_RESERVE_OFS, out_payload_len+2+4);
	/* msg n crc */
	uint16_t crc_ofs = 17 + out_payload_len;

	uint32_t crc = crc32((uint8_t *)rsp, crc_ofs);
	rsp[crc_ofs+3]   = (crc >> 24) & 0xff;
	rsp[crc_ofs+2] = (crc >> 16) & 0xff;
	rsp[crc_ofs+1] = (crc >> 8) & 0xff;
	rsp[crc_ofs+0] = (crc >> 0) & 0xff;
	rsp[62] = ((crc_ofs+4) >> 8) & 0xff;
	rsp[63] = ((crc_ofs+4) >> 0) & 0xff;
	//rt_kprintf("%d crc %x", crc_ofs, crc);
	if (!insert_mem(TYPE_D2H, rsp, 64))
		rt_kprintf("lost d2h packet\r\n");
	notify_event(EVENT_ST2OV);
}
void notify_event(uint32_t _event)
{
	rt_event_send(&event_d2h, _event);
}

void mcu_msg_send(uint8_t *event)
{
	mcu_msg_handler(event);
}

static void mcu_cmd_handler(void *param)
{
	uint8_t msg[64] = {0};
	uint8_t *cmd = {0};
	uint16_t len;
	uint32_t status;
	uint32_t state = EVENT_OV2ST | EVENT_ST2OV;	
	
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
	//normal_timer_init();	
	while (1)
	{
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
		if (rt_event_recv(&event_d2h, state,
					RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
					RT_WAITING_FOREVER,
					(rt_uint32_t *)&status) == RT_EOK)
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
		{
			if (status & EVENT_OV2ST) {
				//rt_kprintf("%ld event 0x%x", read_ts(), status);
				remove_mem(TYPE_H2D, &cmd, &len);
				if (len != 0) {
					//dump_host_cmd("h2d", cmd, len);
					parse_host_cmd(cmd, len, msg);
					//dump_host_cmd("msg", msg, 64);
					mcu_msg_handler(msg);
				}
			}

			if (status & EVENT_ST2OV) {
				//rt_kprintf("%ld event 0x%x", read_ts(), status);
				remove_mem(TYPE_D2H, &cmd, &len);
				if (len > 0) {
					//dump_host_cmd("d2h", cmd, len);
					uart_rsp_out(cmd, len);
				}
			}
		}
	}
}

void protocol_init()
{
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
	rt_memlist_init();
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
	rt_event_init(&event_d2h, "bridge", RT_IPC_FLAG_FIFO);
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
	//rt_thread_t tid = rt_thread_create("proto", mcu_cmd_handler, RT_NULL,
	//		2048, 28, 20);
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
	//rt_thread_startup(tid);
	rt_kprintf("%s %d:\r\n", __func__, __LINE__);
    
    	rt_thread_init(&mcu_thread, "mcu", mcu_cmd_handler, RT_NULL,
                            mcu_stack, sizeof(mcu_stack), RT_THREAD_PRIORITY_MAX / 3, 20);
	rt_thread_startup(&mcu_thread);
}
