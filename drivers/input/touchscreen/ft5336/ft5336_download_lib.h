#ifndef __LINUX_FT5336_DOWNLOAD_LIB_H__
#define __LINUX_FT5336_DOWNLOAD_LIB_H__

typedef int (*FTS_I2c_Read_Function)(unsigned char * , int , unsigned char *, int);
typedef int (*FTS_I2c_Write_Function)(unsigned char * , int );

int Init_I2C_Read_Func(FTS_I2c_Read_Function fpI2C_Read);
int Init_I2C_Write_Func(FTS_I2c_Write_Function fpI2C_Write);

int ft5336_Lib_DownloadMain(unsigned char MainBuf[], unsigned short fwlen);
int ft5336_Lib_ReadMainFlash(unsigned char MainBuf[], unsigned short buflen);
int ft5336_Lib_Enter_Download_Mode(void);
#endif
