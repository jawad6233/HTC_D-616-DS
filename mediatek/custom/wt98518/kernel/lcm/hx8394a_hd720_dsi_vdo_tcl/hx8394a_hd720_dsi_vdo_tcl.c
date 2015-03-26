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
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
	
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_HX8394A                                      0x94

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 

#define LCD_ID_ADC_CHANNEL    12
#define LCD_ID_READ_TIMES    3
#define LCD_ID_VALUE_OFFSET    200

#define TCL_50_INCH_VALUE    0
#define TRULY_50_INCH_VALUE    1244

typedef enum
{
    LCM_TYPE_TCL_50_INCH,
    LCM_TYPE_TRULY_50_INCH,

    LCM_TYPE_MAX,
} LCM_TYPE_E;

static unsigned int lcm_type = 0xFFFF;
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};


static void init_lcm_registers_tcl(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00043902;
    data_array[1] = 0x9483FFB9;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);

    data_array[0] = 0x00113902;
    data_array[1] = 0x002813BA;
    data_array[2] = 0x1000c516;
    data_array[3] = 0x03240fff;
    data_array[4] = 0x20252421;
    data_array[5] = 0x00000008;
    dsi_set_cmdq(&data_array, 6, 1);
    UDELAY(1);

    data_array[0] = 0x00113902;
    data_array[1] = 0x040001B1; //send one param for test
    data_array[2] = 0x11110187;
    data_array[3] = 0x3F3F372f;
    data_array[4] = 0xE6011247;
    data_array[5] = 0x000000E2;
    dsi_set_cmdq(&data_array, 6, 1);
    UDELAY(1);

    data_array[0] = 0x000d3902;
    data_array[1] = 0x08c800B2; 
    data_array[2] = 0x00220004;
    data_array[3] = 0x050000ff;
    data_array[4] = 0x00000019;
    dsi_set_cmdq(&data_array, 5, 1);
    UDELAY(1);

    data_array[0] = 0x00173902;
    data_array[1] = 0x320680B4;
    data_array[2] = 0x15320310;
    data_array[3] = 0x08103208;
    data_array[4] = 0x05430433;
    data_array[5] = 0x06430437;
    data_array[6] = 0x00066161;
    dsi_set_cmdq(&data_array, 7, 1);
    UDELAY(1);

    data_array[0] = 0x00053902;
    data_array[1] = 0x100006BF;
    data_array[2] = 0x00000004;
    dsi_set_cmdq(&data_array, 3, 1);
    UDELAY(1);

    data_array[0] = 0x00033902;
    data_array[1] = 0x00170cc0;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);

    data_array[0] = 0x00033902;
    data_array[1] = 0x000206BF;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);	

    data_array[0] = 0x00023902;
    data_array[1] = 0x00000bB6;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);

    data_array[0] = 0x00213902;
    data_array[1] = 0x000000D5;
    data_array[2] = 0x01000A00;
    data_array[3] = 0x0000CC00;
    data_array[4] = 0x88888800;
    data_array[5] = 0x88888888;
    data_array[6] = 0x01888888;
    data_array[7] = 0x01234567;
    data_array[8] = 0x88888823;
    data_array[9] = 0x00000088;

    dsi_set_cmdq(&data_array, 10, 1);
    UDELAY(1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000009CC;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);

    data_array[0] = 0x00053902;
    data_array[1] = 0x001000C7;
    data_array[2] = 0x00000010;
    dsi_set_cmdq(&data_array, 3, 1);
    UDELAY(1);

    data_array[0] = 0x002B3902; 
    data_array[1] = 0x070500E0; 
    data_array[2] = 0x143F2F2A; 
    data_array[3] = 0x0E0C0733; 
    data_array[4] = 0x12111311; 
    data_array[5] = 0x05001811; 
    data_array[6] = 0x3F2F2A07; 
    data_array[7] = 0x0C073314; 
    data_array[8] = 0x1113110E; 
    data_array[9] = 0x0A181112; 
    data_array[10] = 0x0A120716; 
    data_array[11] = 0x00120716; 
    dsi_set_cmdq(&data_array, 12, 1); 
    UDELAY(1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000032D4;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x00000035;
    dsi_set_cmdq(&data_array, 2, 1);
    UDELAY(1);

    data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(150);

    data_array[0] = 0x00290500; // Display On
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(10);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
	
	params->physical_width = 62.1;
	params->physical_height = 110.4;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity			= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size = 256;

    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;


    params->dsi.word_count = 720*3;	

    params->dsi.vertical_sync_active	=2;//
    params->dsi.vertical_backporch		= 8;//
    params->dsi.vertical_frontporch		= 6;//
    params->dsi.vertical_active_line	= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active	= 40;//
    params->dsi.horizontal_backporch	= 86;//
    params->dsi.horizontal_frontporch	= 86;//
    params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

    params->dsi.pll_select=1;
    params->dsi.PLL_CLOCK=238;
    params->dsi.ssc_range =4;
    params->dsi.ssc_disable = 0;
	
		params->dsi.CLK_TRAIL = 0x52;
		params->dsi.HS_TRAIL = 0x56;	
	
}

static void lcm_adc_id(void)
{
    int lcmid_adc = 0, ret_temp = 0, i = 0;
    int data[4] = {0,0,0,0};
    int res =0;
    
    i = LCD_ID_READ_TIMES;
    
    while (i--) {
        res = IMM_GetOneChannelValue(LCD_ID_ADC_CHANNEL,data,&ret_temp);
        lcmid_adc += ret_temp;
#ifdef BUILD_LK
        printf("Qlw 111[%d] = temp:%d,val:%d\n", i, ret_temp, lcmid_adc);
#else
        printk("Qlw 222[%d] = temp:%d,val:%d\n", i, ret_temp, lcmid_adc);
#endif
    }
    
    lcmid_adc = lcmid_adc/LCD_ID_READ_TIMES;

    if((lcmid_adc > ((int)(TCL_50_INCH_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (TCL_50_INCH_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TCL_50_INCH;
    }
    else if((lcmid_adc > ((int)(TRULY_50_INCH_VALUE - LCD_ID_VALUE_OFFSET))) && (lcmid_adc < (TRULY_50_INCH_VALUE + LCD_ID_VALUE_OFFSET)))
    {
        lcm_type = LCM_TYPE_TRULY_50_INCH;
    }
    else
    {
        lcm_type = LCM_TYPE_MAX;
    }
    
#ifdef BUILD_LK
    printf("Qlw 333 = lcm_type:%d\n", lcm_type);
#else
    printk("Qlw 444 = lcm_type:%d\n", lcm_type);
#endif

    return ;
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    init_lcm_registers_tcl();

    //push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    unsigned int data_array[16];

#if 1	
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
#else
    // Display Off
  //  data_array[0]=0x00280500;
 //   dsi_set_cmdq(&data_array, 1, 1);
 //   MDELAY(120);

    // Sleep In
    data_array[0] = 0x00100500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(150);
#endif
}


static void lcm_resume(void)
{
    unsigned int data_array[16];

#if 1
	SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);
	
	init_lcm_registers_tcl();
#else	
    // Sleep In
    data_array[0] = 0x00110500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(150);

    // Display Off
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array, 1, 1);
    MDELAY(50);
#endif
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);

    unsigned int data_array[16];

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);

    data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    dsi_set_cmdq(&data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(&data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
#if 0
    unsigned int id = 0;
    unsigned char buffer[2];
    unsigned int array[16];  

    SET_RESET_PIN(1);
	MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(10);//Must over 6 ms

    array[0]=0x00043902;
    array[1]=0x9483FFB9;// page enable
    dsi_set_cmdq(&array, 2, 1);
    MDELAY(10);

    read_reg_v2(0xF4, buffer, 2);
    id = buffer[0]; 

#if defined(BUILD_LK)
    printf("%s, LK hx8394a id1 = 0x%08x\n", __func__, id);
#endif

	if(LCM_ID_HX8394A != id)
	return 0;
#endif	
    if(0xFFFF == lcm_type) {
        lcm_adc_id();
    }
    if(LCM_TYPE_TCL_50_INCH == lcm_type)
		return 1;
	else
		return 0;
}


LCM_DRIVER hx8394a_hd720_dsi_vdo_lcm_drv_tcl = 
{
    .name		    = "hx8394a_hd720_dsi_vdo_lcm_tcl",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
