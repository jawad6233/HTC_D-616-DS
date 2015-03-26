/*
 * This file is part of the AP3216C sensor driver for MTK platform.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: cust_alsps.c
 *
 * Summary:
 *	AP3216C hardware defines.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		 Original Creation (Test version:1.0)
 * 06/04/12 YC		 Modify ps threshold field to change to high/low threshold.
 */

#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
 
static struct alsps_hw ap3216c_cust_alsps_hw = {
  .i2c_num    = 2,
  .polling_mode_ps =0,
  .polling_mode_als =1,
  .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
  .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
  //.i2c_addr   = {0x3C, 0x38, 0x3A, 0x00}, //
  .als_level  = { 1, 200, 800, 1500, 2100, 2800, 3300, 3800, 4500, 5500, 6000, 7000, 8800, 20000, 30000},
  .als_value  = { 0,  90,  90,   90,   90,  640,  640,  640, 1800, 1800, 2600, 2600, 2600, 10240, 10240, 10240}, 
  .ps_threshold_high = 120,
  .ps_threshold_low = 54,
};

struct alsps_hw *ap3216c_get_cust_alsps_hw(void) {
  return &ap3216c_cust_alsps_hw;
}

int  CALIBRATION_VALUE_AP3216C = 0x40;  // 64/64, can't be smaller than 0x40

