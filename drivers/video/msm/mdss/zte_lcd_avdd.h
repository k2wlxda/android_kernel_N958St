#ifndef _ZTE_LCD_AVDD_H_
#define _ZTE_LCD_AVDD_H_

void lcd_avdd_on(int vsp_voltage, int vsn_voltage,int vsp_vsn_delay);
void lcd_avdd_off(int vsp_vsn_delay, int avdd_off_delay);

#endif /*_ZTE_LCD_AVDD_H_*/
