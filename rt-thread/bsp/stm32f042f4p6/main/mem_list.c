#include <rtthread.h>
#include <rthw.h>
#include "mem_list.h"

#define MAX_LIST_CNT	10
#define MEM_USED		0x01
#define MEM_FREE		0x00

static rt_list_t rt_uart2hid_list;
static rt_list_t rt_hid2uart_list;
struct rt_mem_list
{
	rt_list_t list;
	rt_uint8_t data[64];
	rt_uint8_t used;
};
typedef struct rt_mem_list *rt_memlist_t;

static struct rt_mem_list uart2hid_mem_list[MAX_LIST_CNT];
static struct rt_mem_list hid2uart_mem_list[MAX_LIST_CNT];

void rt_memlist_init()
{
	int i;

	rt_list_init(&rt_uart2hid_list);
	rt_list_init(&rt_hid2uart_list);

	for (i = 0; i < MAX_LIST_CNT; i++) {
		rt_list_init(&(uart2hid_mem_list[i].list));
		rt_list_init(&(hid2uart_mem_list[i].list));
		uart2hid_mem_list[i].used = MEM_FREE;
		hid2uart_mem_list[i].used = MEM_FREE;
	}
}

rt_bool_t uart2hid_list_isempty()
{
	register rt_base_t level;
	rt_bool_t res = RT_FALSE;

	level = rt_hw_interrupt_disable();
	res = rt_list_isempty(&rt_uart2hid_list);
	rt_hw_interrupt_enable(level);

	return res;
}

rt_bool_t hid2uart_list_isempty()
{
	register rt_base_t level;
	rt_bool_t res = RT_FALSE;

	level = rt_hw_interrupt_disable();
	res = rt_list_isempty(&rt_hid2uart_list);
	rt_hw_interrupt_enable(level);

	return res;
}

rt_bool_t insert_mem(rt_uint8_t type, rt_uint8_t* data)
{
	int i;
	rt_memlist_t ptr;
	rt_list_t *insert;
	register rt_base_t level;

	level = rt_hw_interrupt_disable();

	if (type == TYPE_UART2HID) {
		ptr = uart2hid_mem_list;
		insert = &rt_uart2hid_list;
	} else {
		ptr = hid2uart_mem_list;
		insert = &rt_hid2uart_list;
	}

	for (i = 0; i < MAX_LIST_CNT; i++) {
		if (ptr[i].used == MEM_FREE) {
			ptr[i].used = MEM_USED;
			rt_memcpy(ptr[i].data, data, 64);
			rt_list_insert_after(insert, &(ptr[i].list));
			break;
		}
	}

	rt_hw_interrupt_enable(level);

	if (i == MAX_LIST_CNT)
		return RT_FALSE;

	return RT_TRUE;
}

void remove_mem(rt_uint8_t type, rt_uint8_t **out)
{
	rt_list_t *insert;
	register rt_base_t level;

	level = rt_hw_interrupt_disable();

	if (type == TYPE_UART2HID) {
		insert = &rt_uart2hid_list;
	} else {
		insert = &rt_hid2uart_list;
	}

	*out = RT_NULL;

	if (!rt_list_isempty(insert)) {
		rt_memlist_t to_remove = rt_list_entry(insert->next,
				struct rt_mem_list,
				list);
		to_remove->used = MEM_FREE;
		*out = to_remove->data;
		rt_list_remove(&(to_remove->list));
	}

	rt_hw_interrupt_enable(level);
}
