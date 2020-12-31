#ifndef _MEM_LIST_H
#define _MEM_LIST_H

#ifdef __cplusplus
extern "C" {
#endif

#define TYPE_UART2HID	0x01
#define TYPE_HID2UART	0x00
void rt_memlist_init();
rt_bool_t insert_mem(rt_uint8_t type, rt_uint8_t* data);
void remove_mem(rt_uint8_t type, rt_uint8_t **out);
rt_bool_t hid2uart_list_isempty();
rt_bool_t uart2hid_list_isempty();
void notify_uart2hid();
void notify_hid2uart();

#ifdef __cplusplus
}
#endif

#endif
