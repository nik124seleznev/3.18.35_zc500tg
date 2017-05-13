/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_flashlight_type.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

/*#include <cust_gpio_usage.h>*/
#include <mt_gpio.h>
//#include <gpio_const.h>
/******************************************************************************
 * GPIO configuration
******************************************************************************/
#define GPIO_CAMERA_FLASH_EN_PIN			(GPIO90 | 0x80000000)
#define GPIO_CAMERA_FLASH_EN_PIN_M_CLK		GPIO_MODE_03
#define GPIO_CAMERA_FLASH_EN_PIN_M_EINT		GPIO_MODE_01
#define GPIO_CAMERA_FLASH_EN_PIN_M_GPIO		GPIO_MODE_00
#define GPIO_CAMERA_FLASH_EN_PIN_CLK		CLK_OUT1
#define GPIO_CAMERA_FLASH_EN_PIN_FREQ		GPIO_CLKSRC_NONE

/******************************************************************************
 * Debug configuration
******************************************************************************/

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
#else
	#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_timeOutTimeMs=0;
static BOOL g_is_torch_mode = 0;

static DEFINE_MUTEX(g_strobeSem);

static struct work_struct workTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

#define LEDS_CUSTOM_MODE_THRES 	20

#define FLASH_ON_PULSE 15

int FL_Enable(void)
{
	int i =0;
	PK_DBG("FL_Enable.g_is_torch_mode = %d\n",g_is_torch_mode);	
	if(g_is_torch_mode)
	{
		flashlight_gpio_set(FLASHLIGHT_PIN_FLASH, STATE_LOW);
		flashlight_gpio_set(FLASHLIGHT_PIN_TORCH, STATE_HIGH);
	}
	else
	{
		flashlight_gpio_set(FLASHLIGHT_PIN_TORCH, STATE_LOW);
		flashlight_gpio_set(FLASHLIGHT_PIN_FLASH, STATE_HIGH);
		for(i =0 ; i< FLASH_ON_PULSE;i++)
		{
			udelay(100);
			flashlight_gpio_set(FLASHLIGHT_PIN_FLASH, STATE_LOW);
			udelay(100);
			flashlight_gpio_set(FLASHLIGHT_PIN_FLASH, STATE_HIGH);
		}
	}
	return 0;
}



int FL_Disable(void)
{
		PK_DBG("FL_Disable");
		flashlight_gpio_set(FLASHLIGHT_PIN_TORCH, STATE_LOW);
		flashlight_gpio_set(FLASHLIGHT_PIN_FLASH, STATE_LOW);

	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("FL_dim_duty %d, g_is_torch_mode %d", duty, g_is_torch_mode);
	if(duty == 0)	{
		g_is_torch_mode = 1;		
	}
	else{
		g_is_torch_mode = 0;		
	}
	if(g_timeOutTimeMs < 1001)
	{
		PK_DBG("FL_dim_duty %d > thres %d, timeout %d", duty, g_timeOutTimeMs);	
		g_is_torch_mode = 1;
	}
	PK_DBG("FL_dim_duty %d, g_is_torch_mode %d, timeout %d\n", duty, g_is_torch_mode, g_timeOutTimeMs);
	return 0;
}

int FL_Init(void)
{
	PK_DBG("FL_Init");
	FL_Disable();
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_is_torch_mode = 1;
	return 0;
}

int FL_Uninit(void)
{
	PK_DBG("FL_Uninit");
	FL_Disable();
	g_is_torch_mode = 0;
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift,(int)arg);
    switch(cmd)
    {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
