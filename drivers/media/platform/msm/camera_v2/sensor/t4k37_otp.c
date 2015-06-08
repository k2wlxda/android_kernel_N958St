#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#undef CDBG
//#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define T4K37_OTP_FEATURE 1

#if T4K37_OTP_FEATURE
        //t4k37_otp_init_setting(s_ctrl);
void t4k37_otp_init_setting(struct msm_sensor_ctrl_t *s_ctrl);
#endif

#if T4K37_OTP_FEATURE
typedef struct t4k37_otp_struct 
{
  uint8_t LSC[53];              /* LSC */
  uint8_t AWB[8];               /* AWB */
  uint8_t Module_Info[9];
  uint8_t AF_Macro[2];
  uint8_t AF_Inifity[5];
} st_t4k37_otp;
#define T4K37_OTP_PSEL 0x3502
#define T4K37_OTP_CTRL 0x3500
#define T4K37_OTP_DATA_BEGIN_ADDR 0x3504
#define T4K37_OTP_DATA_END_ADDR 0x3543

static uint16_t t4k37_otp_data[T4K37_OTP_DATA_END_ADDR - T4K37_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
static uint16_t t4k37_otp_data_backup[T4K37_OTP_DATA_END_ADDR - T4K37_OTP_DATA_BEGIN_ADDR + 1] = {0x00};

static uint16_t t4k37_r_golden_value=0x50; //0x91
static uint16_t t4k37_g_golden_value=0x90; //0xA6
static uint16_t t4k37_b_golden_value=0x5d; //0x81

uint16_t  af_macro_value=0;
uint16_t  af_inifity_value=0;
uint16_t  af_otp_status=0;

#if 0
#define SET_T4K37_REG(reg_addr, para)  msm_camera_i2c_write(s_ctrl->sensor_i2c_client,reg_addr,para, MSM_CAMERA_I2C_BYTE_DATA)
#define GET_T4K37_REG(reg_addr,para)   msm_camera_i2c_read(s_ctrl->sensor_i2c_client, reg_addr,&para, MSM_CAMERA_I2C_BYTE_DATA)
#endif
int SET_T4K37_REG(uint32_t addr, uint16_t data, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			addr,
			data,
			MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: write imx214 otp failed\n", __func__);
		return rc;
	}
	CDBG("%s addr=%x, data=%x\n", __func__, addr, data);
	return rc;
}

int GET_T4K37_REG(uint32_t addr, uint16_t *data, struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: read imx214 otp failed\n", __func__);
		return rc;
	}
	chipid = chipid >> 8;
	*data = chipid;
	CDBG("%s: read imx214 otp addr: %x value %x:\n", __func__, addr,
		chipid);
	return chipid;
}
static void t4k37_otp_set_page(struct msm_sensor_ctrl_t *s_ctrl,uint16_t page)
{
    SET_T4K37_REG(T4K37_OTP_PSEL, page,s_ctrl);
}
static void t4k37_otp_access(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t reg_val;
	GET_T4K37_REG(T4K37_OTP_CTRL, &reg_val,s_ctrl);
	SET_T4K37_REG(T4K37_OTP_CTRL, reg_val | 0x80,s_ctrl);
	msleep(10);
}
static void t4k37_otp_read_enble(struct msm_sensor_ctrl_t *s_ctrl,uint8_t enble)
{
    if(enble)
        SET_T4K37_REG(T4K37_OTP_CTRL, 0x01,s_ctrl);
    else
        SET_T4K37_REG(T4K37_OTP_CTRL, 0x00,s_ctrl);
}

static int32_t t4k37_otp_read_data(struct msm_sensor_ctrl_t *s_ctrl, uint16_t* otp_data)
{
    uint16_t i = 0;
    //uint16_t data = 0;


    for (i = 0; i <= (T4K37_OTP_DATA_END_ADDR - T4K37_OTP_DATA_BEGIN_ADDR); i++)
	{
        GET_T4K37_REG(T4K37_OTP_DATA_BEGIN_ADDR+i,&otp_data[i],s_ctrl);
    }

    return 0;
}

static void t4k37_update_awb(struct msm_sensor_ctrl_t *s_ctrl,struct t4k37_otp_struct *p_otp)
{
  uint16_t rg,bg,r_otp,g_otp,b_otp;

  printk("%s:E\n",__func__);
  r_otp=p_otp->AWB[1]+(p_otp->AWB[0]<<8);
  g_otp=(p_otp->AWB[3]+(p_otp->AWB[2]<<8)+p_otp->AWB[5]+(p_otp->AWB[4]<<8))/2;
  b_otp=p_otp->AWB[7]+(p_otp->AWB[6]<<8);
  rg = 256*(t4k37_r_golden_value *g_otp)/(r_otp*t4k37_g_golden_value);
  bg = 256*(t4k37_b_golden_value*g_otp)/(b_otp*t4k37_g_golden_value);

  printk("r_golden=0x%x,g_golden=0x%x, b_golden=0x%0x\n", t4k37_r_golden_value,t4k37_g_golden_value,t4k37_b_golden_value);
  printk("r_otp=0x%x,g_opt=0x%x, b_otp=0x%0x\n", r_otp,g_otp,b_otp);
  printk("rg=0x%x, bg=0x%0x\n", rg,bg);

  SET_T4K37_REG(0x0212, rg >> 8,s_ctrl);
  SET_T4K37_REG(0x0213, rg & 0xff,s_ctrl);

  SET_T4K37_REG(0x0214, bg >> 8,s_ctrl);
  SET_T4K37_REG(0x0215, bg & 0xff,s_ctrl);
  printk("%s:X\n",__func__);
}

static void t4k37_update_lsc(struct msm_sensor_ctrl_t *s_ctrl,struct t4k37_otp_struct *p_otp)
{
  uint16_t addr;
  int i;

 printk("%s:E\n",__func__);
  //set lsc parameters
  addr = 0x323A;
  SET_T4K37_REG(addr, p_otp->LSC[0],s_ctrl);
  addr = 0x323E;
  for(i = 1; i < 53; i++) 
  {
    //printk(" SET LSC[%d], addr:0x%0x, val:0x%0x\n", i, addr, p_otp->LSC[i]);
    SET_T4K37_REG(addr++, p_otp->LSC[i],s_ctrl);
  }
  SET_T4K37_REG(0x3237,0x80,s_ctrl);
   printk("%s:X\n",__func__);
}

static int32_t t4k37_otp_init_lsc_awb(struct msm_sensor_ctrl_t *s_ctrl,struct t4k37_otp_struct *p_otp)
{
  int i,j;
  uint16_t check_sum=0x00;
  printk("%s:E\n",__func__);
  for(i = 3; i >= 0; i--) 
  {
  	t4k37_otp_read_enble(s_ctrl,1);
  	//read data area

	t4k37_otp_set_page(s_ctrl,i);
    t4k37_otp_access(s_ctrl);
	//printk(" otp lsc data area data:%d\n",i);
        t4k37_otp_read_data(s_ctrl, t4k37_otp_data);
	//printk(" otp lsc backup area data:%d\n",i+6);
	t4k37_otp_set_page(s_ctrl,i+6);
	//OTP access
       t4k37_otp_access(s_ctrl);

            t4k37_otp_read_data(s_ctrl, t4k37_otp_data_backup);
  	t4k37_otp_read_enble(s_ctrl,0);

    for(j = 0; j < 64; j++) 
	{
        t4k37_otp_data[j]=t4k37_otp_data[j]|t4k37_otp_data_backup[j];
        }
    if (0 == t4k37_otp_data[0]) 
	{
      continue;
    }
	else 
	{
	  for(j = 2; j < 64; j++) 
	  {
        check_sum=check_sum+t4k37_otp_data[j];
}

	  if((check_sum&0xFF)==t4k37_otp_data[1])
{
	       printk(" otp lsc checksum ok!\n");
		for(j=3;j<=55;j++)
		{
			p_otp->LSC[j-3]=t4k37_otp_data[j];
		}
		for(j=56;j<=63;j++)
		{
			p_otp->AWB[j-56]=t4k37_otp_data[j];
    }
		return 0;
}
	  else
{
		printk(" otp lsc checksum error!\n");

		return -1;
        }
    }

}

  if (i < 0) 
{
    return -1;
    printk(" No otp lsc data on sensor t4k37\n");
  }
    else
  {
    return 0;
  }
}

static int32_t t4k37_otp_init_module_info(struct msm_sensor_ctrl_t *s_ctrl,struct t4k37_otp_struct *p_otp)
{
  int i,pos;
  uint16_t check_sum=0x00;
printk("%s:E\n",__func__);
  //otp enable
  t4k37_otp_read_enble(s_ctrl,1);
  //set page
  t4k37_otp_set_page(s_ctrl,4);
  t4k37_otp_access(s_ctrl);
  //printk(" data area data:\n");
  t4k37_otp_read_data(s_ctrl,t4k37_otp_data);

  t4k37_otp_set_page(s_ctrl,10);
  t4k37_otp_access(s_ctrl);
  t4k37_otp_read_data(s_ctrl,t4k37_otp_data_backup);
  t4k37_otp_read_enble(s_ctrl,0);		
  for(i = 0; i < 64; i++) 
  {
	  t4k37_otp_data[i]=t4k37_otp_data[i]|t4k37_otp_data_backup[i];
	//  printk("t4k37_otp_data[%d] = %x ",i,t4k37_otp_data[i]);
  }

  if(t4k37_otp_data[32])
  {
	  pos=32;
  }
  else if(t4k37_otp_data[0])
  {
  	  pos=0;
  }
  else
  {
  	  printk(" otp no module information!\n");
  	  return -1;
}

  //checking check sum
  for(i = pos+2; i <pos+32; i++) 
{
     check_sum=check_sum+t4k37_otp_data[i];
  }

  if((check_sum&0xFF)==t4k37_otp_data[pos+1])
  {
	  	//printk(" otp module info checksum ok!\n");
		if((t4k37_otp_data[pos+15]==0x00)&&(t4k37_otp_data[pos+16]==0x00)
			&&(t4k37_otp_data[pos+17]==0x00)&&(t4k37_otp_data[pos+18]==0x00)
			&&(t4k37_otp_data[pos+19]==0x00)&&(t4k37_otp_data[pos+20]==0x00)
			&&(t4k37_otp_data[pos+21]==0x00)&&(t4k37_otp_data[pos+22]==0x00))
			return 0;
		
			
		t4k37_r_golden_value=t4k37_otp_data[pos+16]+(t4k37_otp_data[pos+15]<<8);
		t4k37_g_golden_value=(t4k37_otp_data[pos+18]+(t4k37_otp_data[pos+17]<<8)+t4k37_otp_data[pos+20]+(t4k37_otp_data[pos+19]<<8))/2;
		t4k37_b_golden_value=t4k37_otp_data[pos+22]+(t4k37_otp_data[pos+21]<<8);

              printk(" otp module info checksum sucesse!\n");
        
		return 0;
  }
  else
  {
	printk(" otp module info checksum error!\n");
	return -1;
        }
    }


int32_t  t4k37_read_AF_OTP(struct msm_sensor_ctrl_t *s_ctrl,uint16_t *af_macro,uint16_t*af_inifity)
 {
    int j;
    uint16_t  sum=0;
    int  macro_checksum_status= 0;
    int  inifity_checksum_status= 0;
  
   	t4k37_otp_read_enble(s_ctrl,1);
   	//read data area
  
 	t4k37_otp_set_page(s_ctrl,5);
        t4k37_otp_access(s_ctrl);
 	//printk(" otp lsc data area data:%d\n",i);
         t4k37_otp_read_data(s_ctrl, t4k37_otp_data);
 //   for (i = 0; i <= (T4K37_OTP_DATA_END_ADDR - T4K37_OTP_DATA_BEGIN_ADDR); i++)
 //        printk("t4k37_read_page=\t %x \t %x \n",(0x3504+i), t4k37_otp_data[i]);	
    for(j=3;j>=0;j--)
    {	 
          sum= t4k37_otp_data[j*8+2]+t4k37_otp_data[j*8+3]+t4k37_otp_data[j*8+4]+t4k37_otp_data[j*8+5]
		  	+t4k37_otp_data[j*8+6]+t4k37_otp_data[j*8+7];
		  	
   //      printk("t4k37_otp_data[%d] = 0x %x,t4k37_otp_data[%d]= 0x%x,t4k37_otp_data[%d]= 0x%x  \n",
//		  ( j*8),t4k37_otp_data[j*8],(j*8+3) ,t4k37_otp_data[j*8+3],(j*8+4) ,t4k37_otp_data[j*8+4]);
        if((t4k37_otp_data[j*8]==1)&&
	    ((sum%256==t4k37_otp_data[j*8+1])||(t4k37_otp_data[j*8+1] ==t4k37_otp_data[j*8+4])))
         { 
            *af_macro= ((t4k37_otp_data[j*8+3]<<8)|t4k37_otp_data[j*8+4]);
	    macro_checksum_status= 1;
            break;
 	 }	   
    }
    for(j=7;j>=4;j--)
    {	
       sum= t4k37_otp_data[j*8+2]+t4k37_otp_data[j*8+3]+t4k37_otp_data[j*8+4]+t4k37_otp_data[j*8+5]
		     +t4k37_otp_data[j*8+6]+t4k37_otp_data[j*8+7];      
       //      printk("t4k37_otp_data[%d] = 0x %x,t4k37_otp_data[%d]= 0x%x,t4k37_otp_data[%d]= 0x%x  \n",
	//	  ( j*8),t4k37_otp_data[j*8],(j*8+4) ,t4k37_otp_data[j*8+4],(j*8+5) ,t4k37_otp_data[j*8+5]);
        if((t4k37_otp_data[j*8]==1)&&
	    ((sum%256==t4k37_otp_data[j*8+1])||(t4k37_otp_data[j*8+1] ==t4k37_otp_data[j*8+5])))
         { 
            *af_inifity=( (t4k37_otp_data[j*8+4]<<8)|(t4k37_otp_data[j*8+5]));
	     inifity_checksum_status=1;
            break;
 	 }	   
    }
  
    printk("t4k37_read_page  af_macro = %x ,af_inifity = %x \n",*af_macro,*af_inifity);
    
   if(macro_checksum_status ==1 &&inifity_checksum_status==1 )    
   	return 1;
   else
   	return 0;
 }


st_t4k37_otp t4k37_data;
static int t4k37_otp_flag = 0;
static int  t4k37_awb_lsc_cal = 0;
int t4k37_otp_flag_rc = 0xff;

void t4k37_otp_init_setting(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;

	if (t4k37_otp_flag == 0)
	{
	       printk("  %s: %d  get t4k37 otp information \n",__func__,__LINE__);
		rc=t4k37_otp_init_module_info(s_ctrl,&t4k37_data);

	    	t4k37_otp_flag_rc=t4k37_otp_init_lsc_awb(s_ctrl,&t4k37_data);
		t4k37_otp_flag = 1;

	}

	if((t4k37_otp_flag_rc==0x00)&&(t4k37_awb_lsc_cal==0))
	{ 
	      printk("  %s: %d  t4k37 awb and lsc calibration \n",__func__,__LINE__);
		t4k37_update_lsc(s_ctrl,&t4k37_data);
		t4k37_update_awb(s_ctrl,&t4k37_data);
		af_otp_status= t4k37_read_AF_OTP( s_ctrl, &af_macro_value, &af_inifity_value);
		printk("%s:af_otp_status= %d,  af_macro_value= 0x%x, af_inifity_value = 0x%x \n",
		                __func__,af_otp_status,af_macro_value,af_inifity_value);		
		t4k37_awb_lsc_cal=1;
       }

    return;
}
#endif





