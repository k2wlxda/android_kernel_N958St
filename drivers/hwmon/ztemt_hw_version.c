/******************************************************************************

  Copyright (C), 2001-2014, ZTEMT Co., Ltd.

 ******************************************************************************
  File Name     : ztemt_hw_version.c
  Version       : Initial Draft
  Author        : LiuYongfeng
  Created       : 2014/6/5
  Last Modified :
  Description   : hardware version
  Function List :
  History       :
  1.Date        : 2014/6/5
    Author      : LiuYongfeng
    Modification: Created file

******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/string.h>

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

//#define HW_DEBUG

#ifdef HW_DEBUG
#define hw_debug(fmt, args...) printk(KERN_INFO "[HW_VER]"fmt,##args)
#else
#define hw_debug(fmt, args...) do {} while(0)
#endif
#define QPNP_ZTEMT_HW_VERSION_DEV_NAME "qcom,qpnp-ztemt_hw_version"
#define HW_VER_MAX_LEN 100

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/
/**
 * struct qpnp_chg_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 */
struct qpnp_ztemt_hw_version_chip {
	struct device	     *dev;
	struct spmi_device	 *spmi;
	struct work_struct	 work;
	struct mutex	     lock;
};

struct ztemt_hw_info {
	char hw_ver[20];
	char operator[20];
	char project[20];
};

static struct ztemt_hw_info ztemt_board_info = {
	.hw_ver = "UNKNOWN",
	.operator = "UNKNOWN",
	.project = "UNKNOWN",	
};

static int __init ztemt_hw_ver_setup(char *str)
{
	char *buf = NULL;
	char *s = NULL;
	char token[3][20];
	int i = 0;
	
	if(str == NULL)
		return -1;
	/*ZTE_N918X,CM,ZTE_N918X*/
	hw_debug("*********liu.yongfeng str = %s*************\n",str);

	buf = str;

	if (strnlen(str, HW_VER_MAX_LEN)){

		for(s=buf; *s; s++){
			if(*s == ','){
				*s = '\0';
				strncpy(token[i],buf,s-buf+1);
				buf = s+1;
				i++;
			}
		}
		strncpy(token[i],buf,s-buf+1);
		
		/*PROJECT NAME:ZTE_N918X -> ZTE N918X*/
		if(strnlen(token[2],HW_VER_MAX_LEN)){
			for(s=token[2]; *s; s++){
				if(*s == '_')
					*s = ' ';
			}
		}//end of if
	}//end of if

	
	strcpy(ztemt_board_info.hw_ver,token[0]);//ZTE_N918X
	strcpy(ztemt_board_info.operator,token[1]);//CM
	strcpy(ztemt_board_info.project,token[2]);//ZTE N918X

	hw_debug("*********liu.yongfeng hw ver = %s*************\n",ztemt_board_info.hw_ver);
	hw_debug("*********liu.yongfeng operator = %s*************\n",ztemt_board_info.operator);
	hw_debug("*********liu.yongfeng project = %s*************\n",ztemt_board_info.project);
	return 0;
}
__setup("BOARD_INFO=", ztemt_hw_ver_setup);


static ssize_t ztemt_hw_version_show(struct device *dev,
				struct device_attribute *attr, char*buf)
{
	hw_debug("%s hw_ver = %s\n",__func__,ztemt_board_info.hw_ver);
	return sprintf(buf,"%s",ztemt_board_info.hw_ver);
}

static ssize_t ztemt_hw_operators_show(struct device *dev,
				struct device_attribute *attr, char*buf)
{
	hw_debug("%s operator = %s\n",__func__,ztemt_board_info.operator);
	return sprintf(buf,"%s",ztemt_board_info.operator);
}

static ssize_t ztemt_hw_projects_show(struct device *dev,
				struct device_attribute *attr, char*buf)
{
	hw_debug("%s project = %s\n",__func__,ztemt_board_info.project);
	return sprintf(buf,"%s",ztemt_board_info.project);
}

static DEVICE_ATTR(ztemt_hw_version,0664,ztemt_hw_version_show,NULL);

static DEVICE_ATTR(ztemt_hw_operators,0664,ztemt_hw_operators_show,NULL);

static DEVICE_ATTR(ztemt_hw_projects,0664,ztemt_hw_projects_show,NULL);

static struct attribute *dev_ztemt_hw_attributes[] = {
	&dev_attr_ztemt_hw_operators.attr,
	&dev_attr_ztemt_hw_version.attr,
	&dev_attr_ztemt_hw_projects.attr,
	NULL
};

static struct attribute_group dev_attr_ztemt_hw_info_group = {
	.attrs = dev_ztemt_hw_attributes
};

static int qpnp_ztemt_hw_version_probe(struct spmi_device *spmi)
{
	struct qpnp_ztemt_hw_version_chip	*chip;
	int rc = 0;
	hw_debug("[hw]%s begins\n",__func__);
	chip = devm_kzalloc(&spmi->dev,
			sizeof(struct qpnp_ztemt_hw_version_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s : kzalloc() failed.\n",__func__);
		return -ENOMEM;
	}
    dev_set_drvdata(&spmi->dev, chip);
	chip->dev = &(spmi->dev);
	chip->spmi = spmi;

	rc = sysfs_create_group(&chip->dev->kobj,&dev_attr_ztemt_hw_info_group);
	if(rc)
		printk("%s sysfs_create_file failed\n",__func__);

	hw_debug("[hw]%s end\n",__func__);
	return rc;
}

static int qpnp_ztemt_hw_version_remove(struct spmi_device *spmi)
{
	struct qpnp_ztemt_hw_version_chip *chip = dev_get_drvdata(&spmi->dev);
	devm_kfree(&spmi->dev,chip);  
	return 0;
}


static struct of_device_id qpnp_ztemt_hw_version_match_table[] = {
	{ .compatible = QPNP_ZTEMT_HW_VERSION_DEV_NAME, },
	{}
};


static struct spmi_driver qpnp_ztemt_hw_version_driver = {

	.driver		= {
		.name	= QPNP_ZTEMT_HW_VERSION_DEV_NAME,
		.of_match_table = qpnp_ztemt_hw_version_match_table,
	},
	.probe		= qpnp_ztemt_hw_version_probe,
	.remove		= qpnp_ztemt_hw_version_remove,
};

static int __init qpnp_ztemt_hw_version_init(void)
{
	hw_debug("[hw]%s \n",__func__);
	return spmi_driver_register(&qpnp_ztemt_hw_version_driver);
}
module_init(qpnp_ztemt_hw_version_init);

static void __exit qpnp_ztemt_hw_version_exit(void)
{
	spmi_driver_unregister(&qpnp_ztemt_hw_version_driver);
}
module_exit(qpnp_ztemt_hw_version_exit);
	

MODULE_DESCRIPTION("QPNP ZTEMT HARDWARE VERSION");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:Board Hardware version");
