/*
 *akm_data fliter added by ZTEMT
 */
#ifndef SENSOR_FLITER_WORK_H
#define SENSOR_FLITER_WORK_H 

//#define ZTEMT_FLITER_ON
#ifdef ZTEMT_FLITER_ON

#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>



//#define DBG_ON
#define SENSOR_CACHE_LENGTH 17
#define NUM_AXES 3


extern int data_fifo_cache[NUM_AXES][SENSOR_CACHE_LENGTH];
extern uint32_t cache_index;
extern bool start_flag;      


void sensor_fliter_init(void);
int  sensor_find_index(int *dptr,
                       uint8_t len,
                       uint8_t *pmax_idx,
                       uint8_t *pmin_idx);
void sensor_filter_data_start(int *dptr);

#endif
#endif
