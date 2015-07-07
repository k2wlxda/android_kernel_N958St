#ifndef ZTE_LCD_LOG
#define ZTE_LCD_LOG


#define LCD_LOG_TAG "LCD"
#define LCD_LOG_ON
#define LCD_DEBUG_ON

#ifdef  LCD_LOG_ON
#define LCD_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, LCD_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #ifdef  LCD_DEBUG_ON
#define LCD_LOG_DEBUG(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] "  fmt, LCD_LOG_TAG, __FUNCTION__, __LINE__, ##args)                                              
    #else
#define LCD_LOG_DEBUG(fmt, args...)
    #endif
#else
#define LCD_LOG_ERROR(fmt, args...)
#define LCD_LOG_DEBUG(fmt, args...)
#endif


#endif //ZTE_LCD_LOG
