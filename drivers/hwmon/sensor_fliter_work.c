/*
 *implementation of data fliter
 */
 
#include "sensor_fliter_work.h"
#include <linux/mutex.h>


#ifdef ZTEMT_FLITER_ON
DEFINE_MUTEX(lock);

int data_fifo_cache[NUM_AXES][SENSOR_CACHE_LENGTH];
uint32_t cache_index;
bool start_flag;


void sensor_fliter_init() 
{
    mutex_lock(&lock);
    start_flag = false;
    cache_index = 0;
    mutex_unlock(&lock);
}

int sensor_find_index(int *dptr,
    uint8_t len,
    uint8_t *pmax_idx,
    uint8_t *pmin_idx)
{
    uint8_t i;

    *pmax_idx = 0;
    *pmin_idx = 0;
    
    if(1  == len) {
         *pmax_idx = 0;
         *pmin_idx = 0;
    }
    
    if(2 == len) {
        
       if (dptr[0] > dptr[1]) {
         *pmax_idx = 0;
         *pmin_idx = 1;
       } else {
         *pmax_idx = 1;
         *pmin_idx = 0;
       }
    }
    
    for(i = 0; i < len; i += 2) {
       if ((dptr[i] < dptr[i+1]) && (i+1 < len)) {
         int tmp = dptr[i];
         dptr[i] = dptr[i+1];
         dptr[i+1] = tmp;
       }
    }
    
    for(i = 0; i < len; i += 2) {
       if (dptr[i] > dptr[(*pmax_idx)])
          *pmax_idx = i;         
    }
    
    for(i = 1; i < len; i+=2) {
       if (dptr[i] < dptr[(*pmin_idx)])
          *pmin_idx = i; 
    }
    
    if ((*pmax_idx < 0) || (*pmax_idx > len) || 
       (*pmin_idx < 0) || (*pmin_idx > len)) {
        return  -1;
    }
    return 0;
}

void sensor_filter_data_start(int *dptr)
{
    int data_sum[NUM_AXES];
    uint8_t i = 0,j = 0;
    uint8_t max_idx = 0,min_idx = 0;
    int err=-1;
#ifdef DBG_ON
    printk(KERN_ERR"akm 09911 before:dptr[0]=%d,dptr[1]=%d,dptr[2]=%d\n",\
    dptr[0],dptr[1],dptr[2]);
#endif   
    memset(data_sum,0,NUM_AXES*sizeof(int));

    for(i = 0; i < NUM_AXES; i++) {
        data_fifo_cache[i][cache_index] = dptr[i];
    }
    
    mutex_lock(&lock);
    cache_index++;
    cache_index = cache_index % SENSOR_CACHE_LENGTH;
    if (cache_index >= SENSOR_CACHE_LENGTH-1) {
         start_flag = true;
    }
    mutex_unlock(&lock);
  
    if(start_flag) { 
        
      for(i = 0; i < NUM_AXES; i++) { 
        
          err = sensor_find_index(&data_fifo_cache[i][0],SENSOR_CACHE_LENGTH,&max_idx,&min_idx);
          if (err < 0) {
               max_idx = 0;
               min_idx = 0;
          }
            
          for(j = 0; j < SENSOR_CACHE_LENGTH; j++) 
          {
              data_sum[i] += data_fifo_cache[i][j];
          }  
         
          data_sum[i] = data_sum[i] - data_fifo_cache[i][min_idx] - data_fifo_cache[i][max_idx]; 

          dptr[i] = data_sum[i] / (SENSOR_CACHE_LENGTH-2);
      }
      
    } else {
    
      for(i = 0;i < NUM_AXES; i++) { 
         for(j = 0;j < cache_index; j++) {
              data_sum[i] += data_fifo_cache[i][j];
         }  
         dptr[i] = data_sum[i] / (cache_index+1);
      }
      
    }
#ifdef DBG_ON
    printk(KERN_ERR"akm 09911 after:dptr[0]=%d,dptr[1]=%d,dptr[2]=%d\n",\
    dptr[0],dptr[1],dptr[2]);
#endif
}
#endif
