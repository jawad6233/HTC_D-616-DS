
/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************//*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 07 11 2011 jun.pei
 * [ALPS00059464] NT99252 sensor check in
 * .
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "nt99252yuv_Sensor.h"
#include "nt99252yuv_Camera_Sensor_para.h"
#include "nt99252yuv_CameraCustomized.h"

#define NT99252YUV_DEBUG
#ifdef NT99252YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

//#define NT99252_CMCC //1:CMCC_version

static DEFINE_SPINLOCK(NT99252_yuv_drv_lock);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static kal_uint16 Reg32f2 = 0x80;

kal_uint16 NT99252_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 3,NT99252_WRITE_ID);
	return TRUE;

}
kal_uint16 NT99252_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,NT99252_WRITE_ID);
    return get_byte;
}

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0
#define Support720p 1
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* follow is define by jun
********************************************************************************/
MSDK_SENSOR_CONFIG_STRUCT NT99252SensorConfigData;

static struct NT99252_sensor_STRUCT NT99252_sensor;
static kal_uint32 NT99252_zoom_factor = 0; 
static int sensor_id_fail = 0;	

static void NT99252_Init_Parameter(void)
{
    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.first_init = KAL_TRUE;
    NT99252_sensor.pv_mode = KAL_TRUE;
    NT99252_sensor.night_mode = KAL_FALSE;
    NT99252_sensor.MPEG4_Video_mode = KAL_FALSE;

    NT99252_sensor.cp_pclk = NT99252_sensor.pv_pclk;

    NT99252_sensor.pv_dummy_pixels = 0;
    NT99252_sensor.pv_dummy_lines = 0;
    NT99252_sensor.cp_dummy_pixels = 0;
    NT99252_sensor.cp_dummy_lines = 0;

    NT99252_sensor.sceneMode=0;
    NT99252_sensor.isoSpeed=0;
    NT99252_sensor.wb = 0;
    NT99252_sensor.exposure = 0;
    NT99252_sensor.effect = 0;
    NT99252_sensor.banding = AE_FLICKER_MODE_50HZ;

    NT99252_sensor.pv_line_length = 640;
    NT99252_sensor.pv_frame_height = 480;
    NT99252_sensor.cp_line_length = 640;
    NT99252_sensor.cp_frame_height = 480;
    spin_unlock(&NT99252_yuv_drv_lock);    
}


static void NT99252_AWB_enable(kal_bool AWB_enable)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = NT99252_read_cmos_sensor(0x3201);
	
	if (AWB_enable == KAL_TRUE)
	{
		NT99252_write_cmos_sensor(0x3201, (temp_AWB_reg | 0x10));
	}
	else
	{
		NT99252_write_cmos_sensor(0x3201, (temp_AWB_reg & (~0x10)));
	}

}


static kal_uint16 NT99252_power_on(void)
{
    kal_uint16 NT99252_sensor_id = 0;
    spin_lock(&NT99252_yuv_drv_lock);
    //NT99252_sensor.pv_pclk = 48000000;
    spin_unlock(&NT99252_yuv_drv_lock);
    //Software Reset
    NT99252_write_cmos_sensor(0x3021,0x01);

    /* Read Sensor ID  */
    NT99252_sensor_id = ( NT99252_read_cmos_sensor(0x3000)<< 8 )|NT99252_read_cmos_sensor(0x3001);


    SENSORDB("[NT99252YUV]:read Sensor ID:%x\n",NT99252_sensor_id);	
    return NT99252_sensor_id;
}

void NT99252_Initial_Setting(void)
{
  NT99252_write_cmos_sensor(0x32f0,0x02); 
	NT99252_write_cmos_sensor(0x3069,0x01);                                         
	NT99252_write_cmos_sensor(0x306a,0x03); //driving   //0x03                            
	NT99252_write_cmos_sensor(0x302A,0x00); //20140210 
	NT99252_write_cmos_sensor(0x302C,0x09);
	NT99252_write_cmos_sensor(0x302D,0x01);                                        
	NT99252_write_cmos_sensor(0x301F,0x80);                                         
	NT99252_write_cmos_sensor(0x303e,0x01); //dpc on //off = 0x01                               
	NT99252_write_cmos_sensor(0x3082,0x03);                                         
	NT99252_write_cmos_sensor(0x303f,0x0e);                                         
	NT99252_write_cmos_sensor(0x3051,0xE8);                                         
	NT99252_write_cmos_sensor(0x320A,0x00);                                         
	NT99252_write_cmos_sensor(0x302E,0x01); 
	NT99252_write_cmos_sensor(0x3100,0x01); //20140210                                        
	NT99252_write_cmos_sensor(0x3101,0x80);                                         
	NT99252_write_cmos_sensor(0x3104,0x03);                                         
	NT99252_write_cmos_sensor(0x3105,0x03);                                         
	NT99252_write_cmos_sensor(0x3106,0x0D);                                         
	NT99252_write_cmos_sensor(0x310A,0x62);                                         
	NT99252_write_cmos_sensor(0x310D,0x60);                                         
	NT99252_write_cmos_sensor(0x3111,0x5B);                                         
	NT99252_write_cmos_sensor(0x3131,0x58); 
	NT99252_write_cmos_sensor(0x3127,0x01);
	NT99252_write_cmos_sensor(0x3112,0x63); //20140210                                          
	NT99252_write_cmos_sensor(0x3133,0x00);                                         
	NT99252_write_cmos_sensor(0x3134,0x02); 
	NT99252_write_cmos_sensor(0x3135,0x5A);	
	
	NT99252_write_cmos_sensor(0x3290,0x01); // init R_AWB                           
  NT99252_write_cmos_sensor(0x3291,0x80); // init AWB                             
  NT99252_write_cmos_sensor(0x3296,0x01); // init B_AWB                           
  NT99252_write_cmos_sensor(0x3297,0x80); // init AWB  
                                          	            
	NT99252_write_cmos_sensor(0x3127,0x01);  //Lsc_80_ZQ          
	NT99252_write_cmos_sensor(0x3210,0x23);  //Gain0 of R                           
	NT99252_write_cmos_sensor(0x3211,0x27);  //Gain1 of R                           
	NT99252_write_cmos_sensor(0x3212,0x27);  //Gain2 of R                           
	NT99252_write_cmos_sensor(0x3213,0x23);  //Gain3 of R                           
	NT99252_write_cmos_sensor(0x3214,0x1a);  //Gain0 of Gr                          
	NT99252_write_cmos_sensor(0x3215,0x1d);  //Gain1 of Gr                          
	NT99252_write_cmos_sensor(0x3216,0x1c);  //Gain2 of Gr                          
	NT99252_write_cmos_sensor(0x3217,0x18);  //Gain3 of Gr                          
	NT99252_write_cmos_sensor(0x3218,0x1a);  //Gain0 of Gb                          
	NT99252_write_cmos_sensor(0x3219,0x1d);  //Gain1 of Gb                          
	NT99252_write_cmos_sensor(0x321A,0x1c);  //Gain2 of Gb                          
	NT99252_write_cmos_sensor(0x321B,0x18);  //Gain3 of Gb                          
	NT99252_write_cmos_sensor(0x321C,0x14);  //Gain0 of B                           
	NT99252_write_cmos_sensor(0x321D,0x18);  //Gain1 of B                           
	NT99252_write_cmos_sensor(0x321E,0x14);  //Gain2 of B                           
	NT99252_write_cmos_sensor(0x321F,0x11);  //Gain3 of B  
  NT99252_write_cmos_sensor(0x3230,0x02);                                         
	NT99252_write_cmos_sensor(0x3231,0x00);                                         
	NT99252_write_cmos_sensor(0x3232,0x00);                                         
	NT99252_write_cmos_sensor(0x3233,0x05); 
  NT99252_write_cmos_sensor(0x3234,0x00);                                         
	NT99252_write_cmos_sensor(0x3235,0x00);                                         
	NT99252_write_cmos_sensor(0x3236,0x00);                                         
	NT99252_write_cmos_sensor(0x3237,0x00);                                         
	NT99252_write_cmos_sensor(0x3238,0x3f);                                         
	NT99252_write_cmos_sensor(0x3239,0x3f);                                         
	NT99252_write_cmos_sensor(0x323A,0x28); 
	NT99252_write_cmos_sensor(0x3243,0xC3);                                         
	NT99252_write_cmos_sensor(0x3244,0x00);                                         
	NT99252_write_cmos_sensor(0x3245,0x00);
	
	NT99252_write_cmos_sensor(0x3302, 0x00);//115%_20140210
	NT99252_write_cmos_sensor(0x3303, 0x4C);
	NT99252_write_cmos_sensor(0x3304, 0x00);
	NT99252_write_cmos_sensor(0x3305, 0x96);
	NT99252_write_cmos_sensor(0x3306, 0x00);
	NT99252_write_cmos_sensor(0x3307, 0x1D);
	NT99252_write_cmos_sensor(0x3308, 0x07);
	NT99252_write_cmos_sensor(0x3309, 0xB1);
	NT99252_write_cmos_sensor(0x330A, 0x07);
	NT99252_write_cmos_sensor(0x330B, 0x27);
	NT99252_write_cmos_sensor(0x330C, 0x01);
	NT99252_write_cmos_sensor(0x330D, 0x29);
	NT99252_write_cmos_sensor(0x330E, 0x01);
	NT99252_write_cmos_sensor(0x330F, 0x19);
	NT99252_write_cmos_sensor(0x3310, 0x06);
	NT99252_write_cmos_sensor(0x3311, 0xFA);
	NT99252_write_cmos_sensor(0x3312, 0x07);
	NT99252_write_cmos_sensor(0x3313, 0xED);
	                                        
    
	NT99252_write_cmos_sensor(0x3270, 0x00); // Gamma_9712
	NT99252_write_cmos_sensor(0x3271, 0x0c);
	NT99252_write_cmos_sensor(0x3272, 0x18);
	NT99252_write_cmos_sensor(0x3273, 0x31);//0x32
	NT99252_write_cmos_sensor(0x3274, 0x44);
	NT99252_write_cmos_sensor(0x3275, 0x54);
	NT99252_write_cmos_sensor(0x3276, 0x70);
	NT99252_write_cmos_sensor(0x3277, 0x88);
	NT99252_write_cmos_sensor(0x3278, 0x9d);
	NT99252_write_cmos_sensor(0x3279, 0xb0);
	NT99252_write_cmos_sensor(0x327A, 0xcf);
	NT99252_write_cmos_sensor(0x327B, 0xe2);
	NT99252_write_cmos_sensor(0x327C, 0xef);
	NT99252_write_cmos_sensor(0x327D, 0xf7);
	NT99252_write_cmos_sensor(0x327E, 0xff);

  NT99252_write_cmos_sensor(0x3250, 0x01);//(0x3250,0x01);  //CA_CHT834B                              
	NT99252_write_cmos_sensor(0x3251, 0xD0);//(0x3251,0xe0);//0xcc  //                                  
	NT99252_write_cmos_sensor(0x3252, 0xA0);//(0x3252,0xA0);//0x48  //Bottom boundary of R / G          
	NT99252_write_cmos_sensor(0x3253, 0x00);//(0x3253,0x01);  //Top boundary of B / G                   
	NT99252_write_cmos_sensor(0x3254, 0xE0);//(0x3254,0xb1);  //0x70                                    
	NT99252_write_cmos_sensor(0x3255, 0x80);//(0x3255,0x54);  //0x3a Bottom boundary of B / G           
	NT99252_write_cmos_sensor(0x3256, 0x70);//(0x3256,0x70);  //Top boundary of luminance               
	NT99252_write_cmos_sensor(0x3257, 0x10);  
	NT99252_write_cmos_sensor(0x3258, 0x10); //20131018
	NT99252_write_cmos_sensor(0x329C, 0x03); //20131010                                          
	NT99252_write_cmos_sensor(0x329B, 0x01); //Enable WB_GainLimit                      
	NT99252_write_cmos_sensor(0x32A1, 0x00);                                            
	NT99252_write_cmos_sensor(0x32A2, 0x91); //0x91                                     
	NT99252_write_cmos_sensor(0x32A3, 0x01);                                            
	NT99252_write_cmos_sensor(0x32A4, 0x80); //0x46                                     
	NT99252_write_cmos_sensor(0x32A5, 0x01);                                            
	NT99252_write_cmos_sensor(0x32A6, 0x3a);                                            
	NT99252_write_cmos_sensor(0x32A7, 0x02);                                            
	NT99252_write_cmos_sensor(0x32A8, 0x46);                                            
                                                                                                                               
	NT99252_write_cmos_sensor(0x3326,0x0c); // EEXT Sel                             
	NT99252_write_cmos_sensor(0x3327,0x00);                                         
	NT99252_write_cmos_sensor(0x3360,0x08); // IQ Sel                               
	NT99252_write_cmos_sensor(0x3361,0x10);                                         
	NT99252_write_cmos_sensor(0x3362,0x14); 
	                                        
	NT99252_write_cmos_sensor(0x3363,0xb3);// Auto Control                         
	NT99252_write_cmos_sensor(0x3331,0x0a); // EMap                                 
	NT99252_write_cmos_sensor(0x3332,0x80);                                         
	NT99252_write_cmos_sensor(0x3365,0x10);                                         
	NT99252_write_cmos_sensor(0x3366,0x10);                                         
	NT99252_write_cmos_sensor(0x3368,0x38); // Edge Enhance                         
	NT99252_write_cmos_sensor(0x3369,0x28);                                         
	NT99252_write_cmos_sensor(0x336A,0x20);                                         
	NT99252_write_cmos_sensor(0x336B,0x10);                                         
	NT99252_write_cmos_sensor(0x336d,0x18); // DPC                                  
	NT99252_write_cmos_sensor(0x336e,0x12);                                         
	NT99252_write_cmos_sensor(0x336f,0x10);                                         
	NT99252_write_cmos_sensor(0x3370,0x0C);                                         
	NT99252_write_cmos_sensor(0x3379,0x0A); // NR_Comp_Max                          
	NT99252_write_cmos_sensor(0x337A,0x0A);                                         
	NT99252_write_cmos_sensor(0x337B,0x0A);                                         
	NT99252_write_cmos_sensor(0x337C,0x0A);                                         
	NT99252_write_cmos_sensor(0x3371,0x3f); //0x3F // NR_Weight                            
	NT99252_write_cmos_sensor(0x3372,0x3f); //0x3F                                          
	NT99252_write_cmos_sensor(0x3373,0x3F);                                         
	NT99252_write_cmos_sensor(0x3374,0x3F);                                         
	NT99252_write_cmos_sensor(0x33A2,0x00); // AS                                   
	NT99252_write_cmos_sensor(0x33A3,0x30);                                         
	NT99252_write_cmos_sensor(0x33A4,0x01);                                         
  NT99252_write_cmos_sensor(0x33c0,0x03); //Chroma                                
  NT99252_write_cmos_sensor(0x33c9,0xCF);                                         
  NT99252_write_cmos_sensor(0x33ca,0x46);
  
  NT99252_write_cmos_sensor(0x3200,0x3E); 
  NT99252_write_cmos_sensor(0x3201,0x7F);                                            
                           
  NT99252_write_cmos_sensor(0x3012,0x02);                                         
  NT99252_write_cmos_sensor(0x3013,0xC0);                                           
  
  NT99252_write_cmos_sensor(0x32B0,0x66); //Center AE                                                                       
  NT99252_write_cmos_sensor(0x32B1,0x99);                                         
  NT99252_write_cmos_sensor(0x32B2,0x14);                                         
}


/*************************************************************************
* FUNCTION
*	NT99252Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 NT99252Open(void)
{
    spin_lock(&NT99252_yuv_drv_lock);
    sensor_id_fail = 0; 
    spin_unlock(&NT99252_yuv_drv_lock);
    SENSORDB("[Enter]:NT99252 Open func:");

    if (NT99252_power_on() != NT99252_SENSOR_ID) 
    {
        SENSORDB("[NT99252]Error:read sensor ID fail\n");
        spin_lock(&NT99252_yuv_drv_lock);
        sensor_id_fail = 1;
        spin_unlock(&NT99252_yuv_drv_lock);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    /* Apply sensor initail setting*/
    NT99252_Initial_Setting();
    NT99252_Init_Parameter(); 

    SENSORDB("[Exit]:NT99252 Open func\n");     
    return ERROR_NONE;
}	/* NT99252Open() */

/*************************************************************************
* FUNCTION
*	NT99252_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 NT99252_GetSensorID(kal_uint32 *sensorID)
{
    SENSORDB("[Enter]:NT99252 Open func ");
    *sensorID = NT99252_power_on() ;

    if (*sensorID != NT99252_SENSOR_ID) 
    {
        SENSORDB("[NT99252]Error:read sensor ID fail\n");
        spin_lock(&NT99252_yuv_drv_lock);
        sensor_id_fail = 1;
        spin_unlock(&NT99252_yuv_drv_lock);
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }	   

    return ERROR_NONE;    
}   /* NT99252Open  */


/*************************************************************************
* FUNCTION
*	NT99252Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 NT99252Close(void)
{

	return ERROR_NONE;
}	/* NT99252Close() */


static void NT99252_Set_Mirror_Flip(kal_uint16 image_mirror)
{
    /********************************************************
    * Page Mode 0: Reg 0x0011 bit[1:0] = [Y Flip : X Flip]
    * 0: Off; 1: On.
    *********************************************************/ 
    kal_uint16 temp_data;   
    SENSORDB("[Enter]:NT99252 set Mirror_flip func:image_mirror=%d\n",image_mirror);	
    NT99252_write_cmos_sensor(0x3022,0x24);     //Page 0	
    temp_data = (NT99252_read_cmos_sensor(0x3022) & 0xfc);
    spin_lock(&NT99252_yuv_drv_lock);
    //NT99252_sensor.mirror = (NT99252_read_cmos_sensor(0x11) & 0xfc); 
    switch (image_mirror) 
    {
    case IMAGE_NORMAL:
        //NT99252_sensor.mirror |= 0x00;
        temp_data |= 0x02;
        break;
    case IMAGE_H_MIRROR:
        //NT99252_sensor.mirror |= 0x01;
        temp_data |= 0x01;
        break;
    case IMAGE_V_MIRROR:
        //NT99252_sensor.mirror |= 0x02;
        temp_data |= 0x03;
        break;
    case IMAGE_HV_MIRROR:
        //NT99252_sensor.mirror |= 0x03;
        temp_data |= 0x00;
        break;
    default:
        //NT99252_sensor.mirror |= 0x00;
        temp_data |= 0x00;
    }
    NT99252_sensor.mirror = temp_data;
    spin_unlock(&NT99252_yuv_drv_lock);
    NT99252_write_cmos_sensor(0x3022, NT99252_sensor.mirror);
    SENSORDB("[Exit]:NT99252 set Mirror_flip func\n");
}

static kal_uint16 nt99252_Read_Shutter()
{
	int ret = 0;
	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 SensorShutter;
	
	temp_reg1 = NT99252_read_cmos_sensor(0x3013);	
	temp_reg2 = NT99252_read_cmos_sensor(0x3012);	
	SensorShutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	
	return SensorShutter;
}

static kal_uint16 nt99252_Read_FlickerBase()
{
	int ret = 0;
	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 SensorShutter, FlickerBase;
	
	temp_reg1 = NT99252_read_cmos_sensor(0x32C8);
	temp_reg2 = NT99252_read_cmos_sensor(0x32C7);
	temp_reg2 = (temp_reg2 & 0xC0);
	FlickerBase = (temp_reg2 << 2) | (temp_reg1 & 0xFF);
	
	return FlickerBase;
}

static void NT99252_WB_ModeCheck(kal_bool ifPreview)
{	
	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 rGain, bGain;
	kal_uint16 Shutter, FlickerBase;
	
	kal_uint16 rThr = 0xf0;
	kal_uint16 bThr = 0x1c8;
  NT99252_write_cmos_sensor(0x3200, 0x38);
	temp_reg1 = NT99252_read_cmos_sensor(0x3291);	
	temp_reg2 = NT99252_read_cmos_sensor(0x3290);	
	rGain = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
    	
	temp_reg1 = NT99252_read_cmos_sensor(0x3297);	
	temp_reg2 = NT99252_read_cmos_sensor(0x3296);	
	bGain = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
	
	Shutter = nt99252_Read_Shutter();
	FlickerBase = nt99252_Read_FlickerBase();
	
	SENSORDB("[NT99252]:rGain = %x\n",rGain);
	SENSORDB("[NT99252]:bGain = %x\n",bGain);
	SENSORDB("[NT99252]:Shutter = %x\n",Shutter);
	SENSORDB("[NT99252]:flickerbase = %x\n",FlickerBase);
		
	if (ifPreview == KAL_TRUE) // Preview
	{

		NT99252_write_cmos_sensor(0x3302, 0x00); //115% 
		NT99252_write_cmos_sensor(0x3303, 0x4C);        
		NT99252_write_cmos_sensor(0x3304, 0x00);        
		NT99252_write_cmos_sensor(0x3305, 0x96);        
		NT99252_write_cmos_sensor(0x3306, 0x00);        
		NT99252_write_cmos_sensor(0x3307, 0x1D);        
		NT99252_write_cmos_sensor(0x3308, 0x07);        
		NT99252_write_cmos_sensor(0x3309, 0xB1);        
		NT99252_write_cmos_sensor(0x330A, 0x07);        
		NT99252_write_cmos_sensor(0x330B, 0x27);        
		NT99252_write_cmos_sensor(0x330C, 0x01);        
		NT99252_write_cmos_sensor(0x330D, 0x29);        
		NT99252_write_cmos_sensor(0x330E, 0x01);        
		NT99252_write_cmos_sensor(0x330F, 0x19);        
		NT99252_write_cmos_sensor(0x3310, 0x06);        
		NT99252_write_cmos_sensor(0x3311, 0xFA);        
		NT99252_write_cmos_sensor(0x3312, 0x07);        
		NT99252_write_cmos_sensor(0x3313, 0xED);
       
	
	}
	else if((rGain <= rThr)&& (bGain >= bThr) && (Shutter >= FlickerBase) && (ifPreview == KAL_FALSE)) 
	//A-ight capture
	//Condition: R_gain: Low, B_gain_High, ExposureTime >= FlickerBase
	{
		/*
		rGain = (rGain * 96)/100;
		bGain = (bGain * 106)/100;
		NT99252_write_cmos_sensor(0x3290, (rGain >> 8) & 0xFF);
		NT99252_write_cmos_sensor(0x3291, rGain & 0xFF);
		NT99252_write_cmos_sensor(0x3296, (bGain >> 8) & 0xFF);
		NT99252_write_cmos_sensor(0x3297, bGain & 0xFF);
		*/	
		NT99252_write_cmos_sensor(0x3302, 0x00); //CC_105% 
		NT99252_write_cmos_sensor(0x3303, 0x4C);           
		NT99252_write_cmos_sensor(0x3304, 0x00);           
		NT99252_write_cmos_sensor(0x3305, 0x96);           
		NT99252_write_cmos_sensor(0x3306, 0x00);           
		NT99252_write_cmos_sensor(0x3307, 0x1D);           
		NT99252_write_cmos_sensor(0x3308, 0x07);           
		NT99252_write_cmos_sensor(0x3309, 0xB0);           
		NT99252_write_cmos_sensor(0x330A, 0x07);           
		NT99252_write_cmos_sensor(0x330B, 0x02);           
		NT99252_write_cmos_sensor(0x330C, 0x01);           
		NT99252_write_cmos_sensor(0x330D, 0x4E);           
		NT99252_write_cmos_sensor(0x330E, 0x01);           
		NT99252_write_cmos_sensor(0x330F, 0x30);           
		NT99252_write_cmos_sensor(0x3310, 0x06);           
		NT99252_write_cmos_sensor(0x3311, 0xDF);           
		NT99252_write_cmos_sensor(0x3312, 0x07);           
		NT99252_write_cmos_sensor(0x3313, 0xF1);	       

	}	
}

static void NT99252_preview(void)
{    

	NT99252_write_cmos_sensor(0x32FC,0x00); 
	NT99252_write_cmos_sensor(0x32f2,Reg32f2);
	NT99252_write_cmos_sensor(0x32f8,0x01);	
  //[YUYV_1600x1200_10-12fps_60M_2000line]
	NT99252_write_cmos_sensor(0x334A, 0x00); 
	NT99252_write_cmos_sensor(0x334b, 0x7f); 
	NT99252_write_cmos_sensor(0x334c, 0x1f);
	
	NT99252_write_cmos_sensor(0x32BB, 0x87);  //AE Start
	NT99252_write_cmos_sensor(0x32B8, 0x3D); 
	NT99252_write_cmos_sensor(0x32B9, 0x2F); 
	NT99252_write_cmos_sensor(0x32BC, 0x36); 
	NT99252_write_cmos_sensor(0x32BD, 0x3A); 
	NT99252_write_cmos_sensor(0x32BE, 0x32); 
	NT99252_write_cmos_sensor(0x32BF, 0x60); 
	NT99252_write_cmos_sensor(0x32C0, 0x74); 
	NT99252_write_cmos_sensor(0x32C1, 0x74); 
	NT99252_write_cmos_sensor(0x32C2, 0x74); 
	NT99252_write_cmos_sensor(0x32C3, 0x00); 
//NT99252_write_cmos_sensor(0x32C4, 0x1C); 
	NT99252_write_cmos_sensor(0x32C5, 0x1C); 
	NT99252_write_cmos_sensor(0x32C6, 0x1C); 
	NT99252_write_cmos_sensor(0x32C7, 0x00); 
	NT99252_write_cmos_sensor(0x32C8, 0x96); 
	NT99252_write_cmos_sensor(0x32C9, 0x74); 
	NT99252_write_cmos_sensor(0x32CA, 0x90); 
	NT99252_write_cmos_sensor(0x32CB, 0x90); 
	NT99252_write_cmos_sensor(0x32CC, 0x90); 
	NT99252_write_cmos_sensor(0x32CD, 0x90); 
	NT99252_write_cmos_sensor(0x32DB, 0x72);  //AE End
	NT99252_write_cmos_sensor(0x3241, 0x81); 
	NT99252_write_cmos_sensor(0x33A0, 0xB2); 
	NT99252_write_cmos_sensor(0x33A1, 0x50); 
	NT99252_write_cmos_sensor(0x32E0, 0x06);  //Scale Start
	NT99252_write_cmos_sensor(0x32E1, 0x40); 
	NT99252_write_cmos_sensor(0x32E2, 0x04); 
	NT99252_write_cmos_sensor(0x32E3, 0xB0); 
	NT99252_write_cmos_sensor(0x32E4, 0x00); 
	NT99252_write_cmos_sensor(0x32E5, 0x00); 
	NT99252_write_cmos_sensor(0x32E6, 0x00); 
	NT99252_write_cmos_sensor(0x32E7, 0x00);  //Scale End
	NT99252_write_cmos_sensor(0x3200, 0x3E);  //Mode Control
//NT99252_write_cmos_sensor(0x3201, 0x7F);  //Mode Control
//NT99252_write_cmos_sensor(0x302A, 0x00);  //PLL Start
	NT99252_write_cmos_sensor(0x302C, 0x0C); 
	NT99252_write_cmos_sensor(0x302C, 0x09); 
	NT99252_write_cmos_sensor(0x302D, 0x01);  //PLL End
	NT99252_write_cmos_sensor(0x3022, 0x24);  //Timing Start
	NT99252_write_cmos_sensor(0x3023, 0x24); 
	NT99252_write_cmos_sensor(0x3002, 0x00); 
	NT99252_write_cmos_sensor(0x3003, 0x04); 
	NT99252_write_cmos_sensor(0x3004, 0x00); 
	NT99252_write_cmos_sensor(0x3005, 0x04); 
	NT99252_write_cmos_sensor(0x3006, 0x06); 
	NT99252_write_cmos_sensor(0x3007, 0x43); 
	NT99252_write_cmos_sensor(0x3008, 0x04); 
	NT99252_write_cmos_sensor(0x3009, 0xCC); 
	NT99252_write_cmos_sensor(0x300A, 0x07); 
	NT99252_write_cmos_sensor(0x300B, 0xD0); 
	NT99252_write_cmos_sensor(0x300C, 0x04); 
	NT99252_write_cmos_sensor(0x300D, 0xE2); 
	NT99252_write_cmos_sensor(0x300E, 0x06); 
	NT99252_write_cmos_sensor(0x300F, 0x40); 
	NT99252_write_cmos_sensor(0x3010, 0x04); 
	NT99252_write_cmos_sensor(0x3011, 0xB0);  //Timing End
	NT99252_write_cmos_sensor(0x325C, 0x03); 
	NT99252_write_cmos_sensor(0x320A, 0x00); 
	NT99252_write_cmos_sensor(0x3021, 0x02); 
	NT99252_write_cmos_sensor(0x3060, 0x01); 	

}
static void NT99252_video(void)
{

	NT99252_write_cmos_sensor(0x32FC,0x10); 
	NT99252_write_cmos_sensor(0x32f2,0x70);
	NT99252_write_cmos_sensor(0x32f8,0x01);	
  #if 0 //1:video_24fps;  0:video_30fps
  //[YUYV_800x600_24.5fps_68M  line2266 for CMCC]

	NT99252_write_cmos_sensor(0x3326,0x10); 
	NT99252_write_cmos_sensor(0x3331,0x0c); 
	NT99252_write_cmos_sensor(0x3332,0x40);
	NT99252_write_cmos_sensor(0x3368,0x30); // Edge Enhance                         
	NT99252_write_cmos_sensor(0x3369,0x20);                                         
	NT99252_write_cmos_sensor(0x336A,0x18);                                         
	NT99252_write_cmos_sensor(0x336B,0x08);                                         
	NT99252_write_cmos_sensor(0x336d,0x18); // DPC                                  
	NT99252_write_cmos_sensor(0x336e,0x12);                                         
	NT99252_write_cmos_sensor(0x336f,0x10);                                         
	NT99252_write_cmos_sensor(0x3370,0x0C);                                         
	NT99252_write_cmos_sensor(0x3379,0x0A); // NR_Comp_Max                          
	NT99252_write_cmos_sensor(0x337A,0x10);                                         
	NT99252_write_cmos_sensor(0x337B,0x10);                                         
	NT99252_write_cmos_sensor(0x337C,0x1f);                                         
	NT99252_write_cmos_sensor(0x3371,0x38); // NR_Weight                            
	NT99252_write_cmos_sensor(0x3372,0x3f);                                         
	NT99252_write_cmos_sensor(0x3373,0x3F);                                         
	NT99252_write_cmos_sensor(0x3374,0x3F);                                         
	NT99252_write_cmos_sensor(0x33A2,0x00); // AS                                   
	NT99252_write_cmos_sensor(0x33A3,0x40);                                         
	NT99252_write_cmos_sensor(0x33A4,0x00);                                         
	NT99252_write_cmos_sensor(0x33c0,0x0b); //Chroma                                
	NT99252_write_cmos_sensor(0x33c9,0xCF);                                         
	NT99252_write_cmos_sensor(0x33ca,0x36);
	
	NT99252_write_cmos_sensor(0x334A, 0x00); 
	NT99252_write_cmos_sensor(0x334b, 0x7f); 
	NT99252_write_cmos_sensor(0x334c, 0x1f);
	
	NT99252_write_cmos_sensor(0x32BF, 0x60); 
	NT99252_write_cmos_sensor(0x32C0, 0x60); 
	NT99252_write_cmos_sensor(0x32C1, 0x60); 
	NT99252_write_cmos_sensor(0x32C2, 0x60); 
	NT99252_write_cmos_sensor(0x32C3, 0x00); 
	NT99252_write_cmos_sensor(0x32C4, 0x20); 
	NT99252_write_cmos_sensor(0x32C5, 0x20); 
	NT99252_write_cmos_sensor(0x32C6, 0x20); 
	NT99252_write_cmos_sensor(0x32C7, 0x00); 
	NT99252_write_cmos_sensor(0x32C8, 0x99); 
	NT99252_write_cmos_sensor(0x32C9, 0x60); 
	NT99252_write_cmos_sensor(0x32CA, 0x80); 
	NT99252_write_cmos_sensor(0x32CB, 0x80); 
	NT99252_write_cmos_sensor(0x32CC, 0x80); 
	NT99252_write_cmos_sensor(0x32CD, 0x80); 
	NT99252_write_cmos_sensor(0x32DB, 0x73); 
	NT99252_write_cmos_sensor(0x3241, 0x79); 
	NT99252_write_cmos_sensor(0x33A0, 0xB3); 
	NT99252_write_cmos_sensor(0x33A1, 0x20); //0x48
	NT99252_write_cmos_sensor(0x32D0, 0x01); //added by ZQ 20130617
	NT99252_write_cmos_sensor(0x3200, 0x3E); 
//NT99252_write_cmos_sensor(0x3201, 0x3F); 
	NT99252_write_cmos_sensor(0x32e0, 0x03);
	NT99252_write_cmos_sensor(0x32e1, 0x20); 
	NT99252_write_cmos_sensor(0x32e2, 0x02); 
	NT99252_write_cmos_sensor(0x32e3, 0x58); 
	NT99252_write_cmos_sensor(0x32e4, 0x01); 
	NT99252_write_cmos_sensor(0x32e5, 0x00); 
	NT99252_write_cmos_sensor(0x32e6, 0x00); 
	NT99252_write_cmos_sensor(0x32e7, 0x00);

	NT99252_write_cmos_sensor(0x302A, 0x00); 
	NT99252_write_cmos_sensor(0x302C, 0x0C); 
	NT99252_write_cmos_sensor(0x302C, 0x21); 
	NT99252_write_cmos_sensor(0x302D, 0x21); 
	NT99252_write_cmos_sensor(0x3022, 0x24); 
	NT99252_write_cmos_sensor(0x3023, 0x66); 
	NT99252_write_cmos_sensor(0x3002, 0x00); 
	NT99252_write_cmos_sensor(0x3003, 0x04); 
	NT99252_write_cmos_sensor(0x3004, 0x00); 
	NT99252_write_cmos_sensor(0x3005, 0x04); 
	NT99252_write_cmos_sensor(0x3006, 0x06); 
	NT99252_write_cmos_sensor(0x3007, 0x43); 
	NT99252_write_cmos_sensor(0x3008, 0x04); 
	NT99252_write_cmos_sensor(0x3009, 0xCC); 
	NT99252_write_cmos_sensor(0x300A, 0x08); 
	NT99252_write_cmos_sensor(0x300B, 0xb2); 
	NT99252_write_cmos_sensor(0x300C, 0x02); 
	NT99252_write_cmos_sensor(0x300D, 0x6f); 
	NT99252_write_cmos_sensor(0x300E, 0x06); 
	NT99252_write_cmos_sensor(0x300F, 0x40); 
	NT99252_write_cmos_sensor(0x3010, 0x02); 
	NT99252_write_cmos_sensor(0x3011, 0x58); 
	NT99252_write_cmos_sensor(0x32BB, 0x87); 
	NT99252_write_cmos_sensor(0x32B8, 0x3D); 
	NT99252_write_cmos_sensor(0x32B9, 0x2F); 
	NT99252_write_cmos_sensor(0x32BC, 0x36); 
	NT99252_write_cmos_sensor(0x32BD, 0x3A); 
	NT99252_write_cmos_sensor(0x32BE, 0x32); 
	NT99252_write_cmos_sensor(0x325C, 0x03); 
	NT99252_write_cmos_sensor(0x320A, 0x6c); 
	NT99252_write_cmos_sensor(0x3021, 0x06); 
	NT99252_write_cmos_sensor(0x3060, 0x01); 
#else
 //[YUYV_800x600_30.5fps_72M  line1923 for CMCC]

	NT99252_write_cmos_sensor(0x3326,0x10); 
	NT99252_write_cmos_sensor(0x3331,0x0c); 
	NT99252_write_cmos_sensor(0x3332,0x40);
	NT99252_write_cmos_sensor(0x3368,0x30); // Edge Enhance                         
	NT99252_write_cmos_sensor(0x3369,0x20);                                         
	NT99252_write_cmos_sensor(0x336A,0x18);                                         
	NT99252_write_cmos_sensor(0x336B,0x08);                                         
	NT99252_write_cmos_sensor(0x336d,0x18); // DPC                                  
	NT99252_write_cmos_sensor(0x336e,0x12);                                         
	NT99252_write_cmos_sensor(0x336f,0x10);                                         
	NT99252_write_cmos_sensor(0x3370,0x0C);                                         
	NT99252_write_cmos_sensor(0x3379,0x0A); // NR_Comp_Max                          
	NT99252_write_cmos_sensor(0x337A,0x10);                                         
	NT99252_write_cmos_sensor(0x337B,0x10);                                         
	NT99252_write_cmos_sensor(0x337C,0x1f);                                         
	NT99252_write_cmos_sensor(0x3371,0x38); // NR_Weight                            
	NT99252_write_cmos_sensor(0x3372,0x3f);                                         
	NT99252_write_cmos_sensor(0x3373,0x3F);                                         
	NT99252_write_cmos_sensor(0x3374,0x3F);                                         
	NT99252_write_cmos_sensor(0x33A2,0x00); // AS                                   
	NT99252_write_cmos_sensor(0x33A3,0x40);                                         
	NT99252_write_cmos_sensor(0x33A4,0x00);                                         
	NT99252_write_cmos_sensor(0x33c0,0x0b); //Chroma                                
	NT99252_write_cmos_sensor(0x33c9,0xCF);                                         
	NT99252_write_cmos_sensor(0x33ca,0x36);

	
	NT99252_write_cmos_sensor(0x334A, 0x00); 
	NT99252_write_cmos_sensor(0x334b, 0x7f); 
	NT99252_write_cmos_sensor(0x334c, 0x1f);
	
	NT99252_write_cmos_sensor(0x32BF, 0x60); 
	NT99252_write_cmos_sensor(0x32C0, 0x59); 
	NT99252_write_cmos_sensor(0x32C1, 0x59); 
	NT99252_write_cmos_sensor(0x32C2, 0x59); 
	NT99252_write_cmos_sensor(0x32C3, 0x00); 
	NT99252_write_cmos_sensor(0x32C4, 0x20); 
	NT99252_write_cmos_sensor(0x32C5, 0x20); 
	NT99252_write_cmos_sensor(0x32C6, 0x20); 
	NT99252_write_cmos_sensor(0x32C7, 0x00); 
	NT99252_write_cmos_sensor(0x32C8, 0xbb); 
	NT99252_write_cmos_sensor(0x32C9, 0x59); 
	NT99252_write_cmos_sensor(0x32CA, 0x79); 
	NT99252_write_cmos_sensor(0x32CB, 0x79); 
	NT99252_write_cmos_sensor(0x32CC, 0x79); 
	NT99252_write_cmos_sensor(0x32CD, 0x79); 
	NT99252_write_cmos_sensor(0x32DB, 0x77); 
	NT99252_write_cmos_sensor(0x3241, 0x78); 
	NT99252_write_cmos_sensor(0x33A0, 0xB7); 
	NT99252_write_cmos_sensor(0x33A1, 0x39); //0x48
	NT99252_write_cmos_sensor(0x32D0, 0x01); //added by ZQ 20130617
	NT99252_write_cmos_sensor(0x3200, 0x3E); 
//NT99252_write_cmos_sensor(0x3201, 0x3F); 
	NT99252_write_cmos_sensor(0x32e0, 0x03);
	NT99252_write_cmos_sensor(0x32e1, 0x20); 
	NT99252_write_cmos_sensor(0x32e2, 0x02); 
	NT99252_write_cmos_sensor(0x32e3, 0x58); 
	NT99252_write_cmos_sensor(0x32e4, 0x01); 
	NT99252_write_cmos_sensor(0x32e5, 0x00); 
	NT99252_write_cmos_sensor(0x32e6, 0x00); 
	NT99252_write_cmos_sensor(0x32e7, 0x00);

	NT99252_write_cmos_sensor(0x302A, 0x00); 
	NT99252_write_cmos_sensor(0x302C, 0x0C); 
	NT99252_write_cmos_sensor(0x302C, 0x0b); 
	NT99252_write_cmos_sensor(0x302D, 0x01); 
	NT99252_write_cmos_sensor(0x3022, 0x24); //0x27
	NT99252_write_cmos_sensor(0x3023, 0x64); 
	NT99252_write_cmos_sensor(0x3002, 0x00); 
	NT99252_write_cmos_sensor(0x3003, 0x04); 
	NT99252_write_cmos_sensor(0x3004, 0x00); 
	NT99252_write_cmos_sensor(0x3005, 0x04); 
	NT99252_write_cmos_sensor(0x3006, 0x06); 
	NT99252_write_cmos_sensor(0x3007, 0x43); 
	NT99252_write_cmos_sensor(0x3008, 0x04); 
	NT99252_write_cmos_sensor(0x3009, 0xCC); 
	NT99252_write_cmos_sensor(0x300A, 0x07); 
	NT99252_write_cmos_sensor(0x300B, 0x83); 
	NT99252_write_cmos_sensor(0x300C, 0x02); 
	NT99252_write_cmos_sensor(0x300D, 0x65); 
	NT99252_write_cmos_sensor(0x300E, 0x06); 
	NT99252_write_cmos_sensor(0x300F, 0x40); 
	NT99252_write_cmos_sensor(0x3010, 0x02); 
	NT99252_write_cmos_sensor(0x3011, 0x58); 
	NT99252_write_cmos_sensor(0x32BB, 0x87); 
	NT99252_write_cmos_sensor(0x32B8, 0x3D); 
	NT99252_write_cmos_sensor(0x32B9, 0x2F); 
	NT99252_write_cmos_sensor(0x32BC, 0x36); 
	NT99252_write_cmos_sensor(0x32BD, 0x3A); 
	NT99252_write_cmos_sensor(0x32BE, 0x32); 
	NT99252_write_cmos_sensor(0x325C, 0x03); 
	NT99252_write_cmos_sensor(0x320A, 0x6c); 
	NT99252_write_cmos_sensor(0x3021, 0x06); 
	NT99252_write_cmos_sensor(0x3060, 0x01);

#endif

}
static void NT99252_Capture(void)
{
    //[YUYV_1600x1200_5fps_60M_2000line]
	
  NT99252_write_cmos_sensor(0x334A, 0x34); 
	NT99252_write_cmos_sensor(0x334b, 0x14); 
	NT99252_write_cmos_sensor(0x334c, 0x10);
	
	NT99252_write_cmos_sensor(0x32E0, 0x06);  //Scale Start
	NT99252_write_cmos_sensor(0x32E1, 0x40); 
	NT99252_write_cmos_sensor(0x32E2, 0x04); 
	NT99252_write_cmos_sensor(0x32E3, 0xB0); 
	NT99252_write_cmos_sensor(0x32E4, 0x00); 
	NT99252_write_cmos_sensor(0x32E5, 0x00); 
	NT99252_write_cmos_sensor(0x32E6, 0x00); 
	NT99252_write_cmos_sensor(0x32E7, 0x00);  //Scale End
	NT99252_write_cmos_sensor(0x3200, 0x38);  //Mode Control
//NT99252_write_cmos_sensor(0x3201, 0x7F);  //Mode Control
//NT99252_write_cmos_sensor(0x302A, 0x00);  //PLL Start
	NT99252_write_cmos_sensor(0x302C, 0x0C); 
	NT99252_write_cmos_sensor(0x302C, 0x09); 
	NT99252_write_cmos_sensor(0x302D, 0x01);  //PLL End
	NT99252_write_cmos_sensor(0x3022, 0x24);  //Timing Start
	NT99252_write_cmos_sensor(0x3023, 0x24); 
	NT99252_write_cmos_sensor(0x3002, 0x00); 
	NT99252_write_cmos_sensor(0x3003, 0x04); 
	NT99252_write_cmos_sensor(0x3004, 0x00); 
	NT99252_write_cmos_sensor(0x3005, 0x04); 
	NT99252_write_cmos_sensor(0x3006, 0x06); 
	NT99252_write_cmos_sensor(0x3007, 0x43); 
	NT99252_write_cmos_sensor(0x3008, 0x04); 
	NT99252_write_cmos_sensor(0x3009, 0xCC); 
	NT99252_write_cmos_sensor(0x300A, 0x07); 
	NT99252_write_cmos_sensor(0x300B, 0xD0); 
	NT99252_write_cmos_sensor(0x300C, 0x0B); 
	NT99252_write_cmos_sensor(0x300D, 0xB8); 
	NT99252_write_cmos_sensor(0x300E, 0x06); 
	NT99252_write_cmos_sensor(0x300F, 0x40); 
	NT99252_write_cmos_sensor(0x3010, 0x04); 
	NT99252_write_cmos_sensor(0x3011, 0xB0);  //Timing End
	NT99252_write_cmos_sensor(0x325C, 0x03); 
	NT99252_write_cmos_sensor(0x320A, 0x00); 
	NT99252_write_cmos_sensor(0x3021, 0x02); 
	NT99252_write_cmos_sensor(0x3060, 0x01); 

}

static void NT99252_Cal_Min_Frame_Rate(kal_uint16 min_framerate)
{
  
}


static void NT99252_Fix_Video_Frame_Rate(kal_uint16 fix_framerate)
{
 
}


void NT99252_night_mode(kal_bool enable)
{
    SENSORDB("[Enter]NT99252 night mode func:enable = %d\n",enable);
    SENSORDB("NT99252_sensor.video_mode = %d\n",NT99252_sensor.MPEG4_Video_mode); 
    SENSORDB("NT99252_sensor.night_mode = %d\n",NT99252_sensor.night_mode);
    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.night_mode = enable;
    spin_unlock(&NT99252_yuv_drv_lock);

    if(NT99252_sensor.MPEG4_Video_mode == KAL_TRUE)
        return;

    if(enable)
    {
		  NT99252_write_cmos_sensor(0x32C4, 0x28);
			NT99252_write_cmos_sensor(0x302A, 0x04);                         
    }
    else
    {
  
			NT99252_write_cmos_sensor(0x32C4, 0x1c);
			NT99252_write_cmos_sensor(0x302A, 0x00);
    }
}

/*************************************************************************
* FUNCTION
*	NT99252Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 NT99252Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&NT99252_yuv_drv_lock);
    sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR; 
    if(NT99252_sensor.first_init == KAL_TRUE)
    {
        NT99252_sensor.MPEG4_Video_mode = NT99252_sensor.MPEG4_Video_mode;
    }
    else
    {
        NT99252_sensor.MPEG4_Video_mode = !NT99252_sensor.MPEG4_Video_mode;
    }
    spin_unlock(&NT99252_yuv_drv_lock);

    SENSORDB("[Enter]:NT99252 preview func:");		
    SENSORDB("NT99252_sensor.video_mode = %d\n",NT99252_sensor.MPEG4_Video_mode); 

    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.first_init = KAL_FALSE;
    NT99252_sensor.MPEG4_Video_mode = KAL_FALSE;		
    NT99252_sensor.pv_mode = KAL_TRUE;		
    spin_unlock(&NT99252_yuv_drv_lock);

    {   
        SENSORDB("[NT99252]preview mode\n");
        NT99252_WB_ModeCheck(KAL_TRUE);
        NT99252_preview();
    }

    //NT99252_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);

    SENSORDB("[Exit]:NT99252 preview func\n");
    return TRUE; 
}	/* NT99252_Preview */


static UINT32 NT99252Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&NT99252_yuv_drv_lock);
    sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR; 
    if(NT99252_sensor.first_init == KAL_TRUE)
    {
        NT99252_sensor.MPEG4_Video_mode = NT99252_sensor.MPEG4_Video_mode;
    }
    else
    {
        NT99252_sensor.MPEG4_Video_mode = !NT99252_sensor.MPEG4_Video_mode;
    }
    spin_unlock(&NT99252_yuv_drv_lock);

    SENSORDB("[Enter]:NT99252 preview func:");		
    SENSORDB("NT99252_sensor.video_mode = %d\n",NT99252_sensor.MPEG4_Video_mode); 

    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.first_init = KAL_FALSE;
    NT99252_sensor.pv_mode = KAL_FALSE;	
    NT99252_sensor.MPEG4_Video_mode = KAL_TRUE;		
    spin_unlock(&NT99252_yuv_drv_lock);

    {   
        SENSORDB("[NT99252 video mode\n");
        NT99252_video();
    }

    //NT99252_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);

    SENSORDB("[Exit]:NT99252 video func\n");
    return TRUE; 
}	/* NT99252Video */


UINT32 NT99252Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("[NT99252][Enter]NT99252_capture_func\n");
    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.pv_mode = KAL_FALSE;	
    spin_unlock(&NT99252_yuv_drv_lock);
 		{   
        SENSORDB("[NT99252]preview mode\n");
        NT99252_WB_ModeCheck(KAL_FALSE);
        NT99252_Capture();
    }
    return ERROR_NONE;
}	/* NT99252Capture() */


UINT32 NT99252GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[Enter]:NT99252 get Resolution func\n");

    pSensorResolution->SensorFullWidth=NT99252_IMAGE_SENSOR_FULL_WIDTH - 10;  
    pSensorResolution->SensorFullHeight=NT99252_IMAGE_SENSOR_FULL_HEIGHT - 10-10;
    pSensorResolution->SensorPreviewWidth=NT99252_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->SensorPreviewHeight=NT99252_IMAGE_SENSOR_PV_HEIGHT - 12-10;
#ifdef NT99252_CMCC
    pSensorResolution->SensorVideoWidth=NT99252_IMAGE_SENSOR_VIDEO_WIDTH - 16;
    pSensorResolution->SensorVideoHeight=NT99252_IMAGE_SENSOR_VIDEO_HEIGHT - 12-10;
#else
	  pSensorResolution->SensorVideoWidth=NT99252_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->SensorVideoHeight=NT99252_IMAGE_SENSOR_PV_HEIGHT - 12-10;
#endif
    pSensorResolution->Sensor3DFullWidth=NT99252_IMAGE_SENSOR_FULL_WIDTH - 10;  
    pSensorResolution->Sensor3DFullHeight=NT99252_IMAGE_SENSOR_FULL_HEIGHT - 10-10;
    pSensorResolution->Sensor3DPreviewWidth=NT99252_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->Sensor3DPreviewHeight=NT99252_IMAGE_SENSOR_PV_HEIGHT - 12-10;
    pSensorResolution->Sensor3DVideoWidth=NT99252_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->Sensor3DVideoHeight=NT99252_IMAGE_SENSOR_PV_HEIGHT - 12-10;

    SENSORDB("[Exit]:NT99252 get Resolution func\n");	
    return ERROR_NONE;
}	/* NT99252GetResolution() */

UINT32 NT99252GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[Enter]:NT99252 getInfo func:ScenarioId = %d\n",ScenarioId);

    pSensorInfo->SensorPreviewResolutionX=NT99252_IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=NT99252_IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=NT99252_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=NT99252_IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=30;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;//low is to reset 
    pSensorInfo->SensorResetDelayCount=4;  //4ms 
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV; //SENSOR_OUTPUT_FORMAT_YVYU;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;// SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1; 
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;


    pSensorInfo->CaptureDelayFrame = 4; 
    pSensorInfo->PreviewDelayFrame = 1;//10; 
    pSensorInfo->VideoDelayFrame = 0; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA;   	

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:		
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 10;  	
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 10;//1;     			
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 10;//1;     			
        break;
    }
    //	NT99252_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &NT99252SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    SENSORDB("[Exit]:NT99252 getInfo func\n");	
    return ERROR_NONE;
}	/* NT99252GetInfo() */


UINT32 NT99252Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[Enter]:NT99252 Control func:ScenarioId = %d\n",ScenarioId);

    switch (ScenarioId)
    {
#ifdef NT99252_CMCC
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //CMCC_version
    //case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:		
        NT99252Preview(pImageWindow, pSensorConfigData); 
        break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:	
        NT99252Video(pImageWindow, pSensorConfigData); 
        break;
#else
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //normal
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:		
        NT99252Preview(pImageWindow, pSensorConfigData); 
        break;
#endif
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:
        NT99252Capture(pImageWindow, pSensorConfigData); 
        break;
    default:
        break; 
    }

    SENSORDB("[Exit]:NT99252 Control func\n");	
    return TRUE;
}	/* NT99252Control() */

BOOL NT99252_set_scene_mode(UINT16 para)
{
	SENSORDB("NT99252_set_scene_mode=%d",para);	
	spin_lock(&NT99252_yuv_drv_lock);
	NT99252_sensor.sceneMode=para;
	spin_unlock(&NT99252_yuv_drv_lock);

	if(NT99252_sensor.MPEG4_Video_mode == KAL_TRUE)
        return;      
        
    switch (para)
    { 
    	case SCENE_MODE_OFF:
            //normal mode
 			      NT99252_write_cmos_sensor(0x302A, 0x00); 
            NT99252_write_cmos_sensor(0x32C4, 0x1c);
            //chroma component
            NT99252_write_cmos_sensor(0x32F3, 0x80); 
            //AWB offset
            NT99252_write_cmos_sensor(0x32A9, 0x00);
            NT99252_write_cmos_sensor(0x32AA, 0x00); 
            NT99252_write_cmos_sensor(0x32f8, 0x01); 						
			      break;
			case SCENE_MODE_NORMAL:
            //normal mode
 			      NT99252_write_cmos_sensor(0x302A, 0x00); 
            NT99252_write_cmos_sensor(0x32C4, 0x1c);
            //chroma component
            NT99252_write_cmos_sensor(0x32F3, 0x80); 
            //AWB offset
            NT99252_write_cmos_sensor(0x32A9, 0x00);
            NT99252_write_cmos_sensor(0x32AA, 0x00); 
            NT99252_write_cmos_sensor(0x32f8, 0x01); 						
			      break;
		    case SCENE_MODE_NIGHTSCENE:
						//night mode
 			      NT99252_write_cmos_sensor(0x302A, 0x04); 
            NT99252_write_cmos_sensor(0x32C4, 0x28);
            //chroma component
            NT99252_write_cmos_sensor(0x32F3, 0x60); 
            //AWB offset
            NT99252_write_cmos_sensor(0x32A9, 0x00);
            NT99252_write_cmos_sensor(0x32AA, 0x00); 
            NT99252_write_cmos_sensor(0x32f8, 0x01);             		    	
			      break;
        case SCENE_MODE_PORTRAIT:	
            //normal mode
 			      NT99252_write_cmos_sensor(0x302A, 0x00); 
            NT99252_write_cmos_sensor(0x32C4, 0x1c); 
            //chroma component
  			    NT99252_write_cmos_sensor(0x32F3, 0x80);
  			    //AWB offset 
            NT99252_write_cmos_sensor(0x32A9, 0x53);
            NT99252_write_cmos_sensor(0x32AA, 0x01); 
            NT99252_write_cmos_sensor(0x32f8, 0x01);       			 
            break;
        case SCENE_MODE_LANDSCAPE:
            //normal mode
 			      NT99252_write_cmos_sensor(0x302A, 0x00); 
            NT99252_write_cmos_sensor(0x32C4, 0x1c);       	
        	  //chroma component
  			    NT99252_write_cmos_sensor(0x32F3, 0xa0);
  			    //AWB offset 
            NT99252_write_cmos_sensor(0x32A9, 0x23);
            NT99252_write_cmos_sensor(0x32AA, 0x02); 
            NT99252_write_cmos_sensor(0x32f8, 0x01);  			 
            break;
        case SCENE_MODE_SPORTS:
						//normal mode
 			      NT99252_write_cmos_sensor(0x302A, 0x00); 
            NT99252_write_cmos_sensor(0x32C4, 0x1c);
            //chroma component
            NT99252_write_cmos_sensor(0x32F3, 0x80); 
            //AWB offset
            NT99252_write_cmos_sensor(0x32A9, 0x00);
            NT99252_write_cmos_sensor(0x32AA, 0x00); 
            NT99252_write_cmos_sensor(0x32f8, 0x01); 	      	 
        	  break;        	  
        case SCENE_MODE_SUNSET:
            //normal mode
 			      NT99252_write_cmos_sensor(0x302A, 0x00); 
            NT99252_write_cmos_sensor(0x32C4, 0x20);       	
        	  //chroma component
  			    NT99252_write_cmos_sensor(0x32F3, 0x90);
  			    //AWB offset 
            NT99252_write_cmos_sensor(0x32A9, 0x02);
            NT99252_write_cmos_sensor(0x32AA, 0x01); 
            NT99252_write_cmos_sensor(0x32f8, 0x01);
            break;
        case SCENE_MODE_HDR:
            break;
        default:
			return KAL_FALSE;
            break;
    }  
   
	SENSORDB("NT99252_set_scene_mode function:\n ");
	return;
}

BOOL NT99252_set_contrast(UINT16 para)
{   
    SENSORDB("[Enter]NT99252_set_contrast function:\n ");

    switch (para)
    {
        case ISP_CONTRAST_LOW:
 			      NT99252_write_cmos_sensor(0x32FC, 0x20); 
            NT99252_write_cmos_sensor(0x32f2, (Reg32f2-0x20));
            NT99252_write_cmos_sensor(0x32f8, 0x01);            
             break;
        case ISP_CONTRAST_HIGH:
 			      NT99252_write_cmos_sensor(0x32FC, 0xE0); 
            NT99252_write_cmos_sensor(0x32f2, (Reg32f2+0x20));
            NT99252_write_cmos_sensor(0x32f8, 0x01);  
             break;
        case ISP_CONTRAST_MIDDLE:
 			      NT99252_write_cmos_sensor(0x32FC, 0x00); 
            NT99252_write_cmos_sensor(0x32f2, Reg32f2);
            NT99252_write_cmos_sensor(0x32f8, 0x01);
			 break;
        default:
             break;
    }
    SENSORDB("[exit] NT99252_set_contrast function:\n ");
    return;
}

BOOL NT99252_set_saturation(UINT16 para)
{
	  SENSORDB("[enter] NT99252_set_saturation function:\n ");

    switch (para)
    {
        case ISP_SAT_HIGH:
        	  NT99252_write_cmos_sensor(0x32f3,0xa0); //chroma component
						NT99252_write_cmos_sensor(0x33A2,0x20); // AS                                   
						NT99252_write_cmos_sensor(0x33A3,0x10);                                         
						NT99252_write_cmos_sensor(0x33A4,0x01); 
             break;
        case ISP_SAT_LOW:
        	  NT99252_write_cmos_sensor(0x32f3,0x60); //chroma component
						NT99252_write_cmos_sensor(0x33A2,0x00); // AS                                   
						NT99252_write_cmos_sensor(0x33A3,0x30);                                         
						NT99252_write_cmos_sensor(0x33A4,0x01); 
             break;
        case ISP_SAT_MIDDLE:
        	  NT99252_write_cmos_sensor(0x32f3,0x80); //chroma component
						NT99252_write_cmos_sensor(0x33A2,0x00); // AS                                   
						NT99252_write_cmos_sensor(0x33A3,0x30);                                         
						NT99252_write_cmos_sensor(0x33A4,0x01); 
			 break;
        default:
			return KAL_FALSE;
			 break;
    }
	   SENSORDB("[exit] NT99252_set_saturation function:\n ");
     return;
}

BOOL NT99252_set_brightness(UINT16 para)
{
	/*
		kal_uint16 temp_brightness_reg = 0;
	  temp_brightness_reg = NT99252_read_cmos_sensor(0x32fc);
	  
	  SENSORDB("[enter] NT99252_set_brightness function:\n ");
	  //if(NT99252_sensor.MPEG4_Video_mode == KAL_TRUE)
   // return;
    switch (para)
    {
        case ISP_BRIGHT_HIGH:
      			NT99252_write_cmos_sensor(0x32Fc,(temp_brightness_reg+0x20));
						NT99252_write_cmos_sensor(0x32f8,0x01);
             break;
        case ISP_BRIGHT_LOW:
        	  NT99252_write_cmos_sensor(0x32fc,(temp_brightness_reg+0xe0));
						NT99252_write_cmos_sensor(0x32f8,0x01); 
             break;
        case ISP_BRIGHT_MIDDLE:
						NT99252_write_cmos_sensor(0x32fc,temp_brightness_reg);
						NT99252_write_cmos_sensor(0x32f8,0x01);
						break;
        default:

        		break;          
	
    }
	   SENSORDB("[exit] NT99252_set_brightness function:\n ");
	   */
     return;
}

BOOL NT99252_set_iso(UINT16 para)
{
	  SENSORDB("[Enter]NT99252_set_iso func:para = %d\n",para);
	  if(NT99252_sensor.MPEG4_Video_mode == KAL_TRUE)
        return;
    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.isoSpeed = para;
    spin_unlock(&NT99252_yuv_drv_lock);
    switch (para)
    {
        case AE_ISO_AUTO:
        default:
             //ISO AUTO
						NT99252_write_cmos_sensor(0x32C3,0x00); 
						NT99252_write_cmos_sensor(0x32C4,0x1c);                                  
						NT99252_write_cmos_sensor(0x32D0,0x01);     
             break;
        case AE_ISO_100:
             //ISO 100
						NT99252_write_cmos_sensor(0x32C3,0x00); 
						NT99252_write_cmos_sensor(0x32C4,0x10);                                   
						NT99252_write_cmos_sensor(0x32D0,0x01);    
             break;
        case AE_ISO_200:
             //ISO 200
						NT99252_write_cmos_sensor(0x32C3,0x08); 
						NT99252_write_cmos_sensor(0x32C4,0x20);                                    
						NT99252_write_cmos_sensor(0x32D0,0x01);     
             break;
        case AE_ISO_400:
             //ISO 400
						NT99252_write_cmos_sensor(0x32C3,0x10); 
						NT99252_write_cmos_sensor(0x32C4,0x2F);                                    
						NT99252_write_cmos_sensor(0x32D0,0x01);     
             break;
    }
    return;
}

/*************************************************************************
* FUNCTION
*	NT99252_set_param_wb
*
* DESCRIPTION
*	wb setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL NT99252_set_param_wb(UINT16 para)

{
    //This sensor need more time to balance AWB, 
    //we suggest higher fps or drop some frame to avoid garbage color when preview initial
    SENSORDB("[Enter]NT99252 set_param_wb func:para = %d\n",para);

    if(NT99252_sensor.wb == para) return KAL_TRUE;	

    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.wb = para;
    spin_unlock(&NT99252_yuv_drv_lock);
    
    switch (para)
    {          
    case AWB_MODE_AUTO:			
			      NT99252_AWB_enable(KAL_TRUE);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			      NT99252_AWB_enable(KAL_FALSE);
			      NT99252_write_cmos_sensor(0x3290, 0x01); //manual R G B
            NT99252_write_cmos_sensor(0x3291, 0x52);
            NT99252_write_cmos_sensor(0x3296, 0x01);
            NT99252_write_cmos_sensor(0x3297, 0x3A);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny
			      NT99252_AWB_enable(KAL_FALSE);
			      NT99252_write_cmos_sensor(0x3290, 0x01); //manual R G B
            NT99252_write_cmos_sensor(0x3291, 0x20);
            NT99252_write_cmos_sensor(0x3296, 0x01);
            NT99252_write_cmos_sensor(0x3297, 0x70);	
		break;
		
		case AWB_MODE_INCANDESCENT: //A light
			      NT99252_AWB_enable(KAL_FALSE);
						NT99252_write_cmos_sensor(0x3290, 0x00); //manual R G B
            NT99252_write_cmos_sensor(0x3291, 0xc4);
            NT99252_write_cmos_sensor(0x3296, 0x01);
            NT99252_write_cmos_sensor(0x3297, 0xeF);
		break;
		
		case AWB_MODE_TUNGSTEN: //home
			      NT99252_AWB_enable(KAL_FALSE);
						NT99252_write_cmos_sensor(0x3290, 0x01); //manual R G B
            NT99252_write_cmos_sensor(0x3291, 0x00);
            NT99252_write_cmos_sensor(0x3296, 0x02);
            NT99252_write_cmos_sensor(0x3297, 0x30);
		break;
		
		case AWB_MODE_FLUORESCENT:
			      NT99252_AWB_enable(KAL_FALSE);
						NT99252_write_cmos_sensor(0x3290, 0x01); //manual R G B
            NT99252_write_cmos_sensor(0x3291, 0x10);
            NT99252_write_cmos_sensor(0x3296, 0x01);
            NT99252_write_cmos_sensor(0x3297, 0xbf);
		break;
		
		default:
		return FALSE;
		
	
	}

	return TRUE;
} /* NT99252_set_param_wb */

/*************************************************************************
* FUNCTION
*	NT99252_set_param_effect
*
* DESCRIPTION
*	effect setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL NT99252_set_param_effect(UINT16 para)
{
   SENSORDB("[Enter]NT99252 set_param_effect func:para = %d\n",para);
    if(NT99252_sensor.effect == para) return KAL_TRUE;

    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.effect = para;
    spin_unlock(&NT99252_yuv_drv_lock);
    
 	kal_uint32  ret = KAL_TRUE;
#if 1
	switch (para)
	{
		 				case MEFFECT_OFF:
            NT99252_write_cmos_sensor(0x32f1,0x05);
            NT99252_write_cmos_sensor(0x32f4,0x80);
            NT99252_write_cmos_sensor(0x32f5,0x80);
            NT99252_write_cmos_sensor(0x32f8,0x01);
            break;

            case MEFFECT_SEPIA:
            NT99252_write_cmos_sensor(0x32f1,0x02);
            NT99252_write_cmos_sensor(0x32f8,0x01);
            break;

            case MEFFECT_NEGATIVE:
            NT99252_write_cmos_sensor(0x32f1,0x03);
            NT99252_write_cmos_sensor(0x32f8,0x01);
            break;

            case MEFFECT_SEPIAGREEN:
            NT99252_write_cmos_sensor(0x32f1,0x05);
            NT99252_write_cmos_sensor(0x32f4,0x60);
            NT99252_write_cmos_sensor(0x32f5,0x20);
            NT99252_write_cmos_sensor(0x32f8,0x01);
            break;

            case MEFFECT_SEPIABLUE:
            NT99252_write_cmos_sensor(0x32f1,0x05);
            NT99252_write_cmos_sensor(0x32f4,0xf0);
            NT99252_write_cmos_sensor(0x32f5,0x80);
            NT99252_write_cmos_sensor(0x32f8,0x01);
            break;
            case MEFFECT_MONO: //B&W
            NT99252_write_cmos_sensor(0x32f1,0x01);
            NT99252_write_cmos_sensor(0x32f8,0x01);
	    			break;
		default:
			ret = FALSE;
	}
#else
	switch (para)
	{
		case MEFFECT_OFF:
	NT99252_write_cmos_sensor(0x3270, 0x00); // Gamma_9712
	NT99252_write_cmos_sensor(0x3271, 0x0C);
	NT99252_write_cmos_sensor(0x3272, 0x18);
	NT99252_write_cmos_sensor(0x3273, 0x32);
	NT99252_write_cmos_sensor(0x3274, 0x44);
	NT99252_write_cmos_sensor(0x3275, 0x54);
	NT99252_write_cmos_sensor(0x3276, 0x70);
	NT99252_write_cmos_sensor(0x3277, 0x88);
	NT99252_write_cmos_sensor(0x3278, 0x9D);
	NT99252_write_cmos_sensor(0x3279, 0xB0);
	NT99252_write_cmos_sensor(0x327A, 0xCF);
	NT99252_write_cmos_sensor(0x327B, 0xE2);
	NT99252_write_cmos_sensor(0x327C, 0xEF);
	NT99252_write_cmos_sensor(0x327D, 0xF7);
	NT99252_write_cmos_sensor(0x327E, 0xFF);
		break;
		
		case MEFFECT_SEPIA:
	
	NT99252_write_cmos_sensor(0x3270, 0x00); // Gamma3
	NT99252_write_cmos_sensor(0x3271, 0x0B);
	NT99252_write_cmos_sensor(0x3272, 0x16);
	NT99252_write_cmos_sensor(0x3273, 0x2B);
	NT99252_write_cmos_sensor(0x3274, 0x3F);
	NT99252_write_cmos_sensor(0x3275, 0x51);
	NT99252_write_cmos_sensor(0x3276, 0x72);
	NT99252_write_cmos_sensor(0x3277, 0x8F);
	NT99252_write_cmos_sensor(0x3278, 0xA7);
	NT99252_write_cmos_sensor(0x3279, 0xBC);
	NT99252_write_cmos_sensor(0x327A, 0xDC);
	NT99252_write_cmos_sensor(0x327B, 0xF0);
	NT99252_write_cmos_sensor(0x327C, 0xFA);
	NT99252_write_cmos_sensor(0x327D, 0xFE);
	NT99252_write_cmos_sensor(0x327E, 0xFF);


		break;
		
		case MEFFECT_NEGATIVE:
	
	NT99252_write_cmos_sensor(0x3270, 0x08); // Gamma_5
	NT99252_write_cmos_sensor(0x3271, 0x14);
	NT99252_write_cmos_sensor(0x3272, 0x20);
	NT99252_write_cmos_sensor(0x3273, 0x36);
	NT99252_write_cmos_sensor(0x3274, 0x4b);
	NT99252_write_cmos_sensor(0x3275, 0x5D);
	NT99252_write_cmos_sensor(0x3276, 0x7E);
	NT99252_write_cmos_sensor(0x3277, 0x98);
	NT99252_write_cmos_sensor(0x3278, 0xAC);
	NT99252_write_cmos_sensor(0x3279, 0xBD);
	NT99252_write_cmos_sensor(0x327A, 0xD4);
	NT99252_write_cmos_sensor(0x327B, 0xE5);
	NT99252_write_cmos_sensor(0x327C, 0xF0);
	NT99252_write_cmos_sensor(0x327D, 0xF9);
	NT99252_write_cmos_sensor(0x327E, 0xFF);		
		break;
		
		case MEFFECT_SEPIAGREEN:
	NT99252_write_cmos_sensor(0x3270, 0x00); // Gamma_Winson
	NT99252_write_cmos_sensor(0x3271, 0x08);
	NT99252_write_cmos_sensor(0x3272, 0x10);
	NT99252_write_cmos_sensor(0x3273, 0x20);
	NT99252_write_cmos_sensor(0x3274, 0x32);
	NT99252_write_cmos_sensor(0x3275, 0x44);
	NT99252_write_cmos_sensor(0x3276, 0x62);
	NT99252_write_cmos_sensor(0x3277, 0x7b);
	NT99252_write_cmos_sensor(0x3278, 0x92);
	NT99252_write_cmos_sensor(0x3279, 0xA8);
	NT99252_write_cmos_sensor(0x327A, 0xCD);
	NT99252_write_cmos_sensor(0x327B, 0xE7);
	NT99252_write_cmos_sensor(0x327C, 0xF5);
	NT99252_write_cmos_sensor(0x327D, 0xFA);
	NT99252_write_cmos_sensor(0x327E, 0xFF);
		break;
		
		case MEFFECT_SEPIABLUE:
	NT99252_write_cmos_sensor(0x3270, 0x00); // Gamma_9712_2
	NT99252_write_cmos_sensor(0x3271, 0x08);
	NT99252_write_cmos_sensor(0x3272, 0x13);
	NT99252_write_cmos_sensor(0x3273, 0x2b);
	NT99252_write_cmos_sensor(0x3274, 0x41);
	NT99252_write_cmos_sensor(0x3275, 0x53);
	NT99252_write_cmos_sensor(0x3276, 0x72);
	NT99252_write_cmos_sensor(0x3277, 0x8B);
	NT99252_write_cmos_sensor(0x3278, 0x9E);
	NT99252_write_cmos_sensor(0x3279, 0xB3);
	NT99252_write_cmos_sensor(0x327A, 0xCF);
	NT99252_write_cmos_sensor(0x327B, 0xE2);
	NT99252_write_cmos_sensor(0x327C, 0xEF);
	NT99252_write_cmos_sensor(0x327D, 0xF7);
	NT99252_write_cmos_sensor(0x327E, 0xFF);	

		break;

		case MEFFECT_MONO:

	NT99252_write_cmos_sensor(0x3270, 0x00); // Gamma_YM
	NT99252_write_cmos_sensor(0x3271, 0x18);
	NT99252_write_cmos_sensor(0x3272, 0x2E);
	NT99252_write_cmos_sensor(0x3273, 0x50);
	NT99252_write_cmos_sensor(0x3274, 0x6F);
	NT99252_write_cmos_sensor(0x3275, 0x87);
	NT99252_write_cmos_sensor(0x3276, 0xA6);
	NT99252_write_cmos_sensor(0x3277, 0xB7);
	NT99252_write_cmos_sensor(0x3278, 0xC4);
	NT99252_write_cmos_sensor(0x3279, 0xD1);
	NT99252_write_cmos_sensor(0x327A, 0xE1);
	NT99252_write_cmos_sensor(0x327B, 0xEE);
	NT99252_write_cmos_sensor(0x327C, 0xF8);
	NT99252_write_cmos_sensor(0x327D, 0xFF);
	NT99252_write_cmos_sensor(0x327E, 0xFF);
		break;
		default:
			ret = FALSE;
	}

#endif
	return ret;
} /* NT99252_set_param_effect */

/*************************************************************************
* FUNCTION
*	NT99252_set_param_banding
*
* DESCRIPTION
*	banding setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL NT99252_set_param_banding(UINT16 para)
{
    SENSORDB("[Enter]NT99252 set_param_banding func:para = %d\n",para);

    if(NT99252_sensor.banding == para) return KAL_TRUE;

    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.banding = para;
    spin_unlock(&NT99252_yuv_drv_lock);
    
    if(NT99252_sensor.MPEG4_Video_mode == KAL_TRUE)
        return;
    switch (para)
    {
    case AE_FLICKER_MODE_50HZ:
        {
				NT99252_write_cmos_sensor(0x32BF, 0x60);  
				NT99252_write_cmos_sensor(0x32C0, 0x74);  
				NT99252_write_cmos_sensor(0x32C1, 0x74);  
				NT99252_write_cmos_sensor(0x32C2, 0x74);  
				NT99252_write_cmos_sensor(0x32C3, 0x00);  
			//NT99252_write_cmos_sensor(0x32C4, 0x1C);  
			//NT99252_write_cmos_sensor(0x32C5, 0x1C);  
			//NT99252_write_cmos_sensor(0x32C6, 0x1C);  
				NT99252_write_cmos_sensor(0x32C7, 0x00);  
				NT99252_write_cmos_sensor(0x32C8, 0x96);  
				NT99252_write_cmos_sensor(0x32C9, 0x74);  
				NT99252_write_cmos_sensor(0x32CA, 0x90);  
				NT99252_write_cmos_sensor(0x32CB, 0x90);  
				NT99252_write_cmos_sensor(0x32CC, 0x90);  
				NT99252_write_cmos_sensor(0x32CD, 0x90);  
				NT99252_write_cmos_sensor(0x32DB, 0x72);
        
        }
        break;
    case AE_FLICKER_MODE_60HZ:
        {
				NT99252_write_cmos_sensor(0x32BF, 0x60);  
				NT99252_write_cmos_sensor(0x32C0, 0x78);  
				NT99252_write_cmos_sensor(0x32C1, 0x78);  
				NT99252_write_cmos_sensor(0x32C2, 0x78);  
				NT99252_write_cmos_sensor(0x32C3, 0x00);  
			//NT99252_write_cmos_sensor(0x32C4, 0x1C);  
			//NT99252_write_cmos_sensor(0x32C5, 0x1C);  
			//NT99252_write_cmos_sensor(0x32C6, 0x1C);  
				NT99252_write_cmos_sensor(0x32C7, 0x00);  
				NT99252_write_cmos_sensor(0x32C8, 0x7D);  
				NT99252_write_cmos_sensor(0x32C9, 0x78);  
				NT99252_write_cmos_sensor(0x32CA, 0x94);  
				NT99252_write_cmos_sensor(0x32CB, 0x94);  
				NT99252_write_cmos_sensor(0x32CC, 0x94);  
				NT99252_write_cmos_sensor(0x32CD, 0x94);  
				NT99252_write_cmos_sensor(0x32DB, 0x6F); 
				
        }
        break;
    default:
        return KAL_FALSE;
    }
    
    return KAL_TRUE;
} /* NT99252_set_param_banding */




/*************************************************************************
* FUNCTION
*	NT99252_set_param_exposure
*
* DESCRIPTION
*	exposure setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL NT99252_set_param_exposure(UINT16 para)
{
    SENSORDB("[Enter]NT99252 set_param_exposure func:para = %d\n",para);

    if(NT99252_sensor.exposure == para) return KAL_TRUE;

    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.exposure = para;
    spin_unlock(&NT99252_yuv_drv_lock);

    switch (para)
    {
    case AE_EV_COMP_20:  //+2 EV
				NT99252_write_cmos_sensor(0x32F2,0xa0);
				NT99252_write_cmos_sensor(0x32f8,0x01);
        break;     
    case AE_EV_COMP_10:  //+1 EV
				NT99252_write_cmos_sensor(0x32f2,0x90);
				NT99252_write_cmos_sensor(0x32f8,0x01);
        break;       
    case AE_EV_COMP_00:  // +0 EV
				NT99252_write_cmos_sensor(0x32f2,0x80);
				NT99252_write_cmos_sensor(0x32f8,0x01);
        break;       
    case AE_EV_COMP_n10:	// -1 EV		
				NT99252_write_cmos_sensor(0x32f2,0x70);
				NT99252_write_cmos_sensor(0x32f8,0x01);
        break;    
    case AE_EV_COMP_n20:  // -2 EV
				NT99252_write_cmos_sensor(0x32f2,0x60);
				NT99252_write_cmos_sensor(0x32f8,0x01);
        break;
    default:
        return FALSE;
    }
    Reg32f2 =  NT99252_read_cmos_sensor(0x32F2);
    SENSORDB("exit NT99252_set_param_exposure: 0x32F2 = %x\n",Reg32f2);
    return TRUE;	
} /* NT99252_set_param_exposure */

void NT99252_set_AE_mode(UINT32 iPara)
{
}
UINT32 NT99252YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    SENSORDB("[Enter]NT99252YUVSensorSetting func:cmd = %d\n",iCmd);

    switch (iCmd) 
    {
    case FID_SCENE_MODE:	  //added by ZQ 20130614
    	  NT99252_set_scene_mode(iPara);
    	  /*
        if (iPara == SCENE_MODE_OFF)//auto mode
        {
            NT99252_night_mode(FALSE); 
        }
        else if (iPara == SCENE_MODE_NIGHTSCENE)//night mode
        {
            NT99252_night_mode(TRUE); 
        }	
        */
        break;
		case FID_ISP_CONTRAST: //added by ZQ 20130614
        NT99252_set_contrast(iPara);
        break;
    case FID_ISP_SAT:  //added by ZQ 20130614
        NT99252_set_saturation(iPara);
		    break;
    case FID_ISP_BRIGHT:  
        NT99252_set_brightness(iPara);
        break; 
		case FID_AE_ISO:  //added by ZQ 20130614
        NT99252_set_iso(iPara);
        break; 	    
    case FID_AWB_MODE:
        NT99252_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        NT99252_set_param_effect(iPara);
        break;
    case FID_AE_EV:	    	    
        NT99252_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:	    	    	    
        NT99252_set_param_banding(iPara);
        break;
    case FID_ZOOM_FACTOR:
        spin_lock(&NT99252_yuv_drv_lock);
        NT99252_zoom_factor = iPara; 
        spin_unlock(&NT99252_yuv_drv_lock);
        break; 
    case FID_AE_SCENE_MODE: 
        NT99252_set_AE_mode(iPara);
        break; 
    default:
        break;
    }
    return TRUE;
}   /* NT99252YUVSensorSetting */

UINT32 NT99252YUVSetVideoMode(UINT16 u2FrameRate)
{
    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.MPEG4_Video_mode = KAL_TRUE;
    spin_unlock(&NT99252_yuv_drv_lock);
    SENSORDB("[Enter]NT99252 Set Video Mode:FrameRate= %d\n",u2FrameRate);
    SENSORDB("NT99252_sensor.video_mode = %d\n",NT99252_sensor.MPEG4_Video_mode);

    if(u2FrameRate == 30) u2FrameRate = 20;
   
    spin_lock(&NT99252_yuv_drv_lock);
    NT99252_sensor.fix_framerate = u2FrameRate * 10;
    spin_unlock(&NT99252_yuv_drv_lock);
    
    if(NT99252_sensor.fix_framerate <= 300 )
    {
        NT99252_Fix_Video_Frame_Rate(NT99252_sensor.fix_framerate); 
    }
    else 
    {
        SENSORDB("Wrong Frame Rate"); 
    }
        
    return TRUE;
}

void NT99252GetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("NT99252GetAFMaxNumFocusAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

void NT99252GetAEMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{     
    *pFeatureReturnPara32 = 0;    
    SENSORDB("NT99252GetAEMaxNumMeteringAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

void NT99252GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = NT99252_sensor.isoSpeed;
    pExifInfo->AWBMode = NT99252_sensor.wb;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = NT99252_sensor.isoSpeed;
}

UINT32 NT99252FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    //UINT16 u2Temp = 0; 
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=NT99252_IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=NT99252_IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=NT99252_IMAGE_SENSOR_PV_WIDTH;//+NT99252_sensor.pv_dummy_pixels;
        *pFeatureReturnPara16=NT99252_IMAGE_SENSOR_PV_HEIGHT;//+NT99252_sensor.pv_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        //*pFeatureReturnPara32 = NT99252_sensor_pclk/10;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:

        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        NT99252_night_mode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
        break; 
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        NT99252_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = NT99252_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &NT99252SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_GROUP_COUNT:
        // *pFeatureReturnPara32++=0;
        //*pFeatureParaLen=4;
        break; 

    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        NT99252YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;	
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        //NT99252YUVSetVideoMode(*pFeatureData16);
        break; 
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        NT99252_GetSensorID(pFeatureData32); 
        break; 
    case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
        NT99252GetAFMaxNumFocusAreas(pFeatureReturnPara32);            
        *pFeatureParaLen=4;
        break;        
    case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
        NT99252GetAEMaxNumMeteringAreas(pFeatureReturnPara32);            
        *pFeatureParaLen=4;
        break;   
    case SENSOR_FEATURE_GET_EXIF_INFO:
        SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
        SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);          
        NT99252GetExifInfo(*pFeatureData32);
        break;        
    default:
        break;			
    }
    return ERROR_NONE;
}	/* NT99252FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncNT99252=
{
    NT99252Open,
    NT99252GetInfo,
    NT99252GetResolution,
    NT99252FeatureControl,
    NT99252Control,
    NT99252Close
};

UINT32 NT99252_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncNT99252;

    return ERROR_NONE;
}	/* SensorInit() */


