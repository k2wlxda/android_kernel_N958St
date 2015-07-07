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
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

//#define HW_DEBUG

#ifdef HW_DEBUG
#define hw_debug(fmt, args...) printk(KERN_INFO "[HW_VER]"fmt,##args)
#else
#define hw_debug(fmt, args...) do {} while(0)
#endif
#define HW_VER_MAX_LEN 100


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


static ssize_t ztemt_hw_version_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{
	hw_debug("%s hw_ver = %s\n",__func__,ztemt_board_info.hw_ver);
	return snprintf(buf, PAGE_SIZE, "%s\n",	ztemt_board_info.hw_ver);
}

static ssize_t ztemt_hw_operators_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{
	hw_debug("%s operator = %s\n",__func__,ztemt_board_info.operator);
	return snprintf(buf, PAGE_SIZE, "%s\n",	ztemt_board_info.operator);
}

static ssize_t ztemt_hw_projects_show(struct kobject *kobj, 
	struct kobj_attribute *attr, char *buf)
{
	hw_debug("%s project = %s\n",__func__,ztemt_board_info.project);
	return snprintf(buf, PAGE_SIZE, "%s\n",	ztemt_board_info.project);
}


static struct kobj_attribute attrs[] = {
	__ATTR(version, 0664, ztemt_hw_version_show, NULL),
	__ATTR(operators, 0664, ztemt_hw_operators_show, NULL),
	__ATTR(projects, 0664, ztemt_hw_projects_show, NULL),
};


struct kobject *hw_version__kobj;

static int ztemt_hw_version_init(void)
{
	int retval;
	int attr_count = 0;

	hw_version__kobj = kobject_create_and_add("ztemt_hw_version", NULL);
	if (!hw_version__kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		
		retval = sysfs_create_file(hw_version__kobj, &attrs[attr_count].attr);
		if (retval < 0) {
			pr_err("%s: Failed to create sysfs attributes\n", __func__);
			goto err_sys;
		}
	}
	
	pr_info("[hw]%s Done.\n",__func__);

	return retval;
	
err_sys:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(hw_version__kobj, &attrs[attr_count].attr);
	}
	
	kobject_put(hw_version__kobj);
	
	pr_err("[hw]%s init ERR.\n",__func__);

	return retval;
}

static void __exit ztemt_hw_version_exit(void)
{
	int attr_count = 0;
	
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(hw_version__kobj, &attrs[attr_count].attr);
	}
	
	kobject_put(hw_version__kobj);
}


module_init(ztemt_hw_version_init);
module_exit(ztemt_hw_version_exit);


MODULE_DESCRIPTION("QPNP ZTEMT HARDWARE VERSION");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:Board Hardware version");
