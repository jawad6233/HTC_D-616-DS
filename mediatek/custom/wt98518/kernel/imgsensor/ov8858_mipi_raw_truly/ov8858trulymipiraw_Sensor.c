/*******************************************************************************************/
     

/*******************************************************************************************/

#include <linux/videodev2.h>    
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov8858trulymipiraw_Sensor.h"
#include "ov8858trulymipiraw_Camera_Sensor_para.h"
#include "ov8858trulymipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(ov8858trulymipiraw_drv_lock);

//#define OV8858_DEBUG
#ifdef OV8858TRULY_DEBUG
	#define OV8858TRULYDB(fmt, arg...) xlog_printk(ANDROID_LOG_ERROR, "[OV8858TRULYRaw] ",  fmt, ##arg)
#else
	#define OV8858TRULYDB(fmt, arg...)
#endif


kal_uint32 OV8858TRULY_FeatureControl_PERIOD_PixelNum=OV8858TRULY_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV8858TRULY_FeatureControl_PERIOD_LineNum=OV8858TRULY_PV_PERIOD_LINE_NUMS;

UINT16 OV8858TRULY_VIDEO_MODE_TARGET_FPS = 30;

MSDK_SCENARIO_ID_ENUM OV8858TRULYCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT OV8858TRULYSensorConfigData;
static OV8858TRULY_PARA_STRUCT ov8858truly;
kal_uint32 OV8858TRULY_FAC_SENSOR_REG;


SENSOR_REG_STRUCT OV8858TRULYSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV8858TRULYSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;


//TODO~
#define OV8858TRULY_TEST_PATTERN_CHECKSUM 0x47a75476
kal_bool OV8858TRULY_During_testpattern = KAL_FALSE;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV8858TRULY_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV8858TRULYMIPI_WRITE_ID)

kal_uint16 OV8858TRULY_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,OV8858TRULYMIPI_WRITE_ID);
    return get_byte;
}
extern int OV8858_VCM_start,OV8858_VCM_end;
#define OTP_CALIBRATION
#ifdef OTP_CALIBRATION
#define RG_Ratio_Typical 0x143
#define BG_Ratio_Typical 0x114

#define OTP_DRV_START_ADDR 0x7010
#define OTP_DRV_INFO_GROUP_COUNT 3
#define OTP_DRV_INFO_SIZE 5
#define OTP_DRV_AWB_GROUP_COUNT 3
#define OTP_DRV_AWB_SIZE 5
#define OTP_DRV_VCM_GROUP_COUNT 3
#define OTP_DRV_VCM_SIZE 3
#define OTP_DRV_LSC_GROUP_COUNT 3
#define OTP_DRV_LSC_SIZE 110
#define OTP_DRV_LSC_REG_ADDR 0x5800

struct otp_struct {
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
int light_rg;
int light_bg;
int lenc[OTP_DRV_LSC_SIZE ];
int VCM_start;
int VCM_end;
int VCM_dir;
};

//Gionee <zhangpj><2013-08-05> fix CR00839642 <reduce enter preview time> begin
#ifndef ORIGINAL_VERSION	
static struct otp_struct s_otp_wb;
static struct otp_struct s_otp_lenc;
static bool isNeedReadOtp = false;
#endif
//Gionee <zhangpj><2013-08-05> fix CR00839642 <reduce enter preview time> end
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int check_otp_info(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff );
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	flag = OV8858TRULY_read_cmos_sensor(nFlagAddress);
	//select group
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index ==3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV8858TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

static int check_otp_wb(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//select group
	flag = OV8858TRULY_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV8858TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}
static int check_otp_lenc(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE
	+1+OTP_DRV_VCM_GROUP_COUNT*OTP_DRV_VCM_SIZE ;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	flag = OV8858TRULY_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>> 2)& 0x03;
	}
	// clear otp buffer
	OV8858TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

static int check_otp_VCM(int index)
{
	int flag;
	int nFlagAddress= OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV8858TRULY_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//select group
	flag = OV8858TRULY_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV8858TRULY_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

int g_ov8858truly_module_id = 0;
static int read_otp_info(struct otp_struct *otp_ptr)
{
	int i;
	int nFlagAddress = OTP_DRV_START_ADDR;
	int start_addr, end_addr;
	int otp_index;
	int temp;
	
	for(i=1;i<=OTP_DRV_INFO_GROUP_COUNT;i++) {
		temp = check_otp_info(i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>OTP_DRV_AWB_GROUP_COUNT) {
	// no valid module info OTP data
	return 1;
	}
	start_addr = nFlagAddress+1+(otp_index-1)*OTP_DRV_INFO_SIZE;
	end_addr = start_addr+OTP_DRV_INFO_SIZE-1;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	(*otp_ptr).module_integrator_id = OV8858TRULY_read_cmos_sensor(start_addr);
	(*otp_ptr).lens_id = OV8858TRULY_read_cmos_sensor(start_addr + 1);
	(*otp_ptr).production_year = OV8858TRULY_read_cmos_sensor(start_addr + 2);
	(*otp_ptr).production_month = OV8858TRULY_read_cmos_sensor(start_addr + 3);
	(*otp_ptr).production_day = OV8858TRULY_read_cmos_sensor(start_addr + 4);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV8858TRULY_write_cmos_sensor(i, 0x00);
}
	g_ov8858truly_module_id = (*otp_ptr).module_integrator_id;

return 0;
}
static int read_otp_wb(int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	start_addr = nFlagAddress+1+(index-1)* OTP_DRV_AWB_SIZE;
	end_addr = start_addr+OTP_DRV_AWB_SIZE;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	temp = OV8858TRULY_read_cmos_sensor(start_addr + 4);
	(*otp_ptr).rg_ratio = (OV8858TRULY_read_cmos_sensor(start_addr)<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (OV8858TRULY_read_cmos_sensor(start_addr + 1)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (OV8858TRULY_read_cmos_sensor(start_addr + 2) <<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (OV8858TRULY_read_cmos_sensor(start_addr + 3)<<2) + (temp & 0x03);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV8858TRULY_write_cmos_sensor(i, 0x00);
	}

return 0;
}

static int read_otp_VCM(int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_VCM_SIZE;
	end_addr = start_addr+OTP_DRV_VCM_SIZE-1;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//flag and lsb of VCM start code
	temp = OV8858TRULY_read_cmos_sensor(start_addr+2);
	(* otp_ptr).VCM_start = (OV8858TRULY_read_cmos_sensor(start_addr)<<2) | ((temp>>6) & 0x03);
	(* otp_ptr).VCM_end = (OV8858TRULY_read_cmos_sensor(start_addr + 1) << 2) | ((temp>>4) & 0x03);
	(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	
	OV8858_VCM_start = (* otp_ptr).VCM_start;
	OV8858_VCM_end = (* otp_ptr).VCM_end;
	OV8858TRULYDB("jin read_otp_VCM OV8858_VCM_start=%d,OV8858_VCM_end=%d",OV8858_VCM_start,OV8858_VCM_end);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
		OV8858TRULY_write_cmos_sensor(i, 0x00);
	}
return 0;
}
static int read_otp_lenc(int index, struct otp_struct * otp_ptr)
{
	int i;
	int start_addr, end_addr;
	int nFlagAddress= OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT* OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT* OTP_DRV_AWB_SIZE
	+1+OTP_DRV_VCM_GROUP_COUNT* OTP_DRV_VCM_SIZE ;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_LSC_SIZE ;
	end_addr = start_addr+OTP_DRV_LSC_SIZE-1;
	OV8858TRULY_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV8858TRULY_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV8858TRULY_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV8858TRULY_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV8858TRULY_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(10);
	for(i=0; i<OTP_DRV_LSC_SIZE; i++) {
	(* otp_ptr).lenc[i] = OV8858TRULY_read_cmos_sensor(start_addr + i);
	
	OV8858TRULYDB("jin OTP  i is : %d, lenc is : 0x%x \n ",i,(* otp_ptr).lenc[i]);

	}
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV8858TRULY_write_cmos_sensor(i, 0x00);
	}
return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	if (R_gain>0x400) {
	OV8858TRULY_write_cmos_sensor(0x5032, R_gain>>8);
	OV8858TRULY_write_cmos_sensor(0x5033, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
	OV8858TRULY_write_cmos_sensor(0x5034, G_gain>>8);
	OV8858TRULY_write_cmos_sensor(0x5035, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
	OV8858TRULY_write_cmos_sensor(0x5036, B_gain>>8);
	OV8858TRULY_write_cmos_sensor(0x5037, B_gain & 0x00ff);
	}
	return 0;
	// otp_ptr: pointer of otp_struct
}

static int update_lenc(struct otp_struct * otp_ptr)
{
	int i, temp;
	temp = OV8858TRULY_read_cmos_sensor(0x5000);
	temp = 0x80 | temp;
	OV8858TRULY_write_cmos_sensor(0x5000, temp);
	for(i=0;i<OTP_DRV_LSC_SIZE ;i++) {
	OV8858TRULY_write_cmos_sensor(OTP_DRV_LSC_REG_ADDR + i, (*otp_ptr).lenc[i]);
	}
	return 0;
}

// call this function after OV8858TRULY initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_wb()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	int rg=0,bg=0;
	int R_gain, G_gain, B_gain;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for(i=1;i<=OTP_DRV_AWB_GROUP_COUNT;i++) {
	temp = check_otp_wb(i);
	if (temp == 2) {
	otp_index = i;
	break;
	}
	}
	if (i>OTP_DRV_AWB_GROUP_COUNT) {
	// no valid wb OTP data
	return 1;
	}
	//read_otp_info(otp_index, &current_otp);
	read_otp_wb(otp_index, &current_otp);
	if(current_otp.light_rg==0) {
	// no light source information in OTP, light factor = 1
	rg = current_otp.rg_ratio;
	}
	else {
	rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
	}
	if(current_otp.light_bg==0) {
	// not light source information in OTP, light factor = 1
	bg = current_otp.bg_ratio;
	}
	else {
	bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
	}

	OV8858TRULYDB("jin %s rg=0x%x\n ",__FUNCTION__,rg);
	OV8858TRULYDB("jin %s bg=0x%x\n ",__FUNCTION__,bg);

	//calculate G gain
	int nR_G_gain, nB_G_gain, nG_G_gain;
	int nBase_gain;
	nR_G_gain = (RG_Ratio_Typical*1000) / rg;
	nB_G_gain = (BG_Ratio_Typical*1000) / bg;
	nG_G_gain = 1000;
	if (nR_G_gain < 1000 || nB_G_gain < 1000)
	{
	if (nR_G_gain < nB_G_gain)
	nBase_gain = nR_G_gain;
	else
	nBase_gain = nB_G_gain;
	}
	else
	{
	nBase_gain = nG_G_gain;
	}
	R_gain = 0x400 * nR_G_gain / (nBase_gain);
	B_gain = 0x400 * nB_G_gain / (nBase_gain);
	G_gain = 0x400 * nG_G_gain / (nBase_gain);

	OV8858TRULYDB("R_gain=0x%x\n ",R_gain);
	OV8858TRULYDB("B_gain=0x%x\n ",B_gain);
	OV8858TRULYDB("G_gain=0x%x\n ",G_gain);

	update_awb_gain(R_gain, G_gain, B_gain);
	return 0;

}
static int update_otp_VCM()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=OTP_DRV_VCM_GROUP_COUNT;i++) {
		temp = check_otp_VCM(i);	
		OV8858TRULYDB("jin ###update_otp_VCM temp is : %d, i is :%d \n ",temp, i);
		if (temp == 2) {
			otp_index = i;
		break;}
	}
	if (i>OTP_DRV_LSC_GROUP_COUNT) {
	// no valid WB OTP data
	return 1;
	}
	read_otp_VCM(otp_index, &current_otp);
	// success
	return 0;
}

// call this function after OV8858TRULY initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_lenc()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=OTP_DRV_LSC_GROUP_COUNT;i++) {
	temp = check_otp_lenc(i);	
	OV8858TRULYDB("jin ###update_otp_lenc temp is : %d, i is :%d \n ",temp, i);
	if (temp == 2) {
	otp_index = i;
	break;}
	}
	if (i>OTP_DRV_LSC_GROUP_COUNT) {
	// no valid WB OTP data
	return 1;
	}
	read_otp_lenc(otp_index, &current_otp);
	update_lenc(&current_otp);
		update_otp_VCM();
	// success
	return 0;
}


#endif


void OV8858TRULY_Init_Para(void)
{

	spin_lock(&ov8858trulymipiraw_drv_lock);
	ov8858truly.sensorMode = SENSOR_MODE_INIT;
	ov8858truly.OV8858TRULYAutoFlickerMode = KAL_FALSE;
	ov8858truly.OV8858TRULYVideoMode = KAL_FALSE;
	ov8858truly.DummyLines= 0;
	ov8858truly.DummyPixels= 0;
	ov8858truly.pvPclk =  (7200); 
	ov8858truly.videoPclk = (7200);
	ov8858truly.capPclk = (14400);

	ov8858truly.shutter = 0x4C00;
	ov8858truly.ispBaseGain = BASEGAIN;		//64
	ov8858truly.sensorGlobalGain = 0x0200;  //512
	spin_unlock(&ov8858trulymipiraw_drv_lock);
}

kal_uint32 GetOv8858TRULYLineLength(void)
{
	kal_uint32 OV8858TRULY_line_length = 0;
	if ( SENSOR_MODE_PREVIEW == ov8858truly.sensorMode )  
	{
		OV8858TRULY_line_length = OV8858TRULY_PV_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels;
	}
	else if( SENSOR_MODE_VIDEO == ov8858truly.sensorMode ) 
	{
		OV8858TRULY_line_length = OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels;
	}
	else
	{
		OV8858TRULY_line_length = OV8858TRULY_FULL_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels;
	}
	
#ifdef OV8858TRULY_DEBUG
	OV8858TRULYDB("[GetOv8858TRULYLineLength]: ov8858truly.sensorMode = %d, OV8858TRULY_line_length =%d, ov8858truly.DummyPixels = %d\n", ov8858truly.sensorMode,OV8858TRULY_line_length,ov8858truly.DummyPixels);
#endif


    return OV8858TRULY_line_length;

}


kal_uint32 GetOv8858TRULYFrameLength(void)
{
	kal_uint32 OV8858TRULY_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == ov8858truly.sensorMode )  
	{
		OV8858TRULY_frame_length = OV8858TRULY_PV_PERIOD_LINE_NUMS + ov8858truly.DummyLines ;
	}
	else if( SENSOR_MODE_VIDEO == ov8858truly.sensorMode ) 
	{
		OV8858TRULY_frame_length = OV8858TRULY_VIDEO_PERIOD_LINE_NUMS + ov8858truly.DummyLines ;
	}
	else
	{
		OV8858TRULY_frame_length = OV8858TRULY_FULL_PERIOD_LINE_NUMS + ov8858truly.DummyLines ;
	}

#ifdef OV8858TRULY_DEBUG
		OV8858TRULYDB("[GetOv8858TRULYFrameLength]: ov8858truly.sensorMode = %d, OV8858TRULY_frame_length =%d, ov8858truly.DummyLines = %d\n", ov8858truly.sensorMode,OV8858TRULY_frame_length,ov8858truly.DummyLines);
#endif


	return OV8858TRULY_frame_length;
}


kal_uint32 OV8858TRULY_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	
	if (shutter_value <4 ){
		shutter_value = 4;
	}

	
	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

#ifdef OV8858TRULY_DEBUG
			OV8858TRULYDB("[OV8858TRULY_CalcExtra_For_ShutterMargin]: shutter_value = %d, shutterLimitation =%d, extra_lines = %d\n", shutter_value,shutterLimitation,extra_lines);
#endif

    return extra_lines;

}


//TODO~
kal_uint32 OV8858TRULY_CalcFrameLength_For_AutoFlicker(void)
{

    kal_uint32 AutoFlicker_min_framelength = 0;

	switch(OV8858TRULYCurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			AutoFlicker_min_framelength = (ov8858truly.capPclk*10000) /(OV8858TRULY_FULL_PERIOD_LINE_NUMS + ov8858truly.DummyPixels)/OV8858TRULY_AUTOFLICKER_OFFSET_30*10 ;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(OV8858TRULY_VIDEO_MODE_TARGET_FPS==30)
			{
				AutoFlicker_min_framelength = (ov8858truly.videoPclk*10000) /(OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels)/OV8858TRULY_AUTOFLICKER_OFFSET_30*10 ;
			}
			else if(OV8858TRULY_VIDEO_MODE_TARGET_FPS==15)
			{
				AutoFlicker_min_framelength = (ov8858truly.videoPclk*10000) /(OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels)/OV8858TRULY_AUTOFLICKER_OFFSET_15*10 ;
			}
			else
			{
				AutoFlicker_min_framelength = OV8858TRULY_VIDEO_PERIOD_LINE_NUMS + ov8858truly.DummyLines;
			}
			break;
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			AutoFlicker_min_framelength = (ov8858truly.pvPclk*10000) /(OV8858TRULY_PV_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels)/OV8858TRULY_AUTOFLICKER_OFFSET_30*10 ;
			break;
	}

	#ifdef OV8858TRULY_DEBUG 
	OV8858TRULYDB("AutoFlicker_min_framelength =%d,OV8858TRULYCurrentScenarioId =%d\n", AutoFlicker_min_framelength,OV8858TRULYCurrentScenarioId);
	#endif

	return AutoFlicker_min_framelength;

}


void OV8858TRULY_write_shutter(kal_uint32 shutter)
{
	//kal_uint32 min_framelength = OV8858TRULY_PV_PERIOD_PIXEL_NUMS;
	//the init code write as up line;
	//modify it as follow
	kal_uint32 min_framelength = OV8858TRULY_PV_PERIOD_LINE_NUMS;
	kal_uint32 max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	//TODO~
	kal_uint32 read_shutter_1 = 0;
	kal_uint32 read_shutter_2 = 0;
	kal_uint32 read_shutter_3 = 0;

	//TODO~
    if(shutter > 0x90f7)//500ms for capture SaturationGain
    {
    	#ifdef OV8858TRULY_DEBUG
		OV8858TRULYDB("[OV8858TRULY_write_shutter] shutter > 0x90f7 [warn.] shutter=%x, \n", shutter);
		#endif
		shutter = 0x90f7;
    }
	
    line_length  = GetOv8858TRULYLineLength();
	frame_length = GetOv8858TRULYFrameLength();
	
	max_shutter  = frame_length-OV8858TRULY_SHUTTER_MARGIN;

    frame_length = frame_length + OV8858TRULY_CalcExtra_For_ShutterMargin(shutter,max_shutter);
	


	if(ov8858truly.OV8858TRULYAutoFlickerMode == KAL_TRUE)
	{
        min_framelength = OV8858TRULY_CalcFrameLength_For_AutoFlicker();

        if(frame_length < min_framelength)
			frame_length = min_framelength;
	}
	

	spin_lock_irqsave(&ov8858trulymipiraw_drv_lock,flags);
	OV8858TRULY_FeatureControl_PERIOD_PixelNum = line_length;
	OV8858TRULY_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&ov8858trulymipiraw_drv_lock,flags);

	//Set total frame length
	OV8858TRULY_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV8858TRULY_write_cmos_sensor(0x380f, frame_length & 0xFF);
	
	//Set shutter 
	OV8858TRULY_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	OV8858TRULY_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	OV8858TRULY_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	

	#ifdef OV8858TRULY_DEBUG
	OV8858TRULYDB("[OV8858TRULY_write_shutter]ov8858truly write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);
	#endif

}


void OV8858TRULY_SetShutter(kal_uint32 iShutter)
{

   spin_lock(&ov8858trulymipiraw_drv_lock);
   ov8858truly.shutter= iShutter;
   spin_unlock(&ov8858trulymipiraw_drv_lock);

   OV8858TRULY_write_shutter(iShutter);
   return;
}


UINT32 OV8858TRULY_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV8858TRULY_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV8858TRULY_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV8858TRULY_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

static kal_uint16 OV8858TRULYReg2Gain(const kal_uint16 iReg)
{
    kal_uint16 iGain =0; 

	iGain = iReg*BASEGAIN/OV8858TRULY_GAIN_BASE;
	return iGain;
}

static kal_uint16 OV8858TRULYGain2Reg(const kal_uint32 iGain)
{
    kal_uint32 iReg = 0x0000;

	iReg = iGain*OV8858TRULY_GAIN_BASE/BASEGAIN;

    return iReg;
}

void write_OV8858TRULY_gain(kal_uint16 gain)
{
	//kal_uint16 read_gain=0;

	OV8858TRULY_write_cmos_sensor(0x3508,(gain>>8));
	OV8858TRULY_write_cmos_sensor(0x3509,(gain&0xff));

	//read_gain=(((OV8858TRULY_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8858TRULY_read_cmos_sensor(0x3509));
	//OV8858TRULYDB("[OV8858TRULY_SetGain]0x3508|0x3509=0x%x \n",read_gain);

	return;
}

void OV8858TRULY_SetGain(UINT16 iGain)
{
	unsigned long flags;

	
	OV8858TRULYDB("OV8858TRULY_SetGain iGain = %d :\n ",iGain);

	spin_lock_irqsave(&ov8858trulymipiraw_drv_lock,flags);
	ov8858truly.realGain = iGain;
	ov8858truly.sensorGlobalGain = OV8858TRULYGain2Reg(iGain);
	spin_unlock_irqrestore(&ov8858trulymipiraw_drv_lock,flags);
	write_OV8858TRULY_gain(ov8858truly.sensorGlobalGain);
	#ifdef OV8858TRULY_DEBUG
	OV8858TRULYDB(" [OV8858TRULY_SetGain]ov8858truly.sensorGlobalGain=0x%x,ov8858truly.realGain =%x",ov8858truly.sensorGlobalGain,ov8858truly.realGain); 
	#endif
	//temperature test
	//OV8858TRULY_write_cmos_sensor(0x4d12,0x01);
	//OV8858TRULYDB("Temperature read_reg  0x4d13  =%x \n",OV8858TRULY_read_cmos_sensor(0x4d13));
}   

kal_uint16 read_OV8858TRULY_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=(((OV8858TRULY_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8858TRULY_read_cmos_sensor(0x3509));

	spin_lock(&ov8858trulymipiraw_drv_lock);
	ov8858truly.sensorGlobalGain = read_gain;
	ov8858truly.realGain = OV8858TRULYReg2Gain(ov8858truly.sensorGlobalGain);
	spin_unlock(&ov8858trulymipiraw_drv_lock);

	OV8858TRULYDB("ov8858truly.sensorGlobalGain=0x%x,ov8858truly.realGain=%d\n",ov8858truly.sensorGlobalGain,ov8858truly.realGain);

	return ov8858truly.sensorGlobalGain;
}  



static void OV8858TRULY_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == ov8858truly.sensorMode )
	{
		line_length = OV8858TRULY_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858TRULY_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== ov8858truly.sensorMode )
	{
		line_length = OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858TRULY_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = OV8858TRULY_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858TRULY_FULL_PERIOD_LINE_NUMS + iLines;
	}

	spin_lock(&ov8858trulymipiraw_drv_lock);
	OV8858TRULY_FeatureControl_PERIOD_PixelNum = line_length;
	OV8858TRULY_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&ov8858trulymipiraw_drv_lock);

	//Set total frame length
	OV8858TRULY_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV8858TRULY_write_cmos_sensor(0x380f, frame_length & 0xFF);
	//Set total line length
	OV8858TRULY_write_cmos_sensor(0x380c, (line_length >> 8) & 0xFF);
	OV8858TRULY_write_cmos_sensor(0x380d, line_length & 0xFF);

	#ifdef OV8858TRULY_DEBUG
	OV8858TRULYDB(" [OV8858TRULY_SetDummy]ov8858truly.sensorMode = %d, line_length = %d,iPixels = %d, frame_length =%d, iLines = %d\n",ov8858truly.sensorMode, line_length,iPixels, frame_length, iLines); 
	#endif

}   


#if 1
void OV8858TRULY_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV8858TRULYSensorReg[i].Addr; i++)
    {
        OV8858TRULY_write_cmos_sensor(OV8858TRULYSensorReg[i].Addr, OV8858TRULYSensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8858TRULYSensorReg[i].Addr; i++)
    {
        OV8858TRULY_write_cmos_sensor(OV8858TRULYSensorReg[i].Addr, OV8858TRULYSensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV8858TRULY_write_cmos_sensor(OV8858TRULYSensorCCT[i].Addr, OV8858TRULYSensorCCT[i].Para);
    }
}

void OV8858TRULY_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=OV8858TRULYSensorReg[i].Addr; i++)
    {
         temp_data = OV8858TRULY_read_cmos_sensor(OV8858TRULYSensorReg[i].Addr);
		 spin_lock(&ov8858trulymipiraw_drv_lock);
		 OV8858TRULYSensorReg[i].Para =temp_data;
		 spin_unlock(&ov8858trulymipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8858TRULYSensorReg[i].Addr; i++)
    {
        temp_data = OV8858TRULY_read_cmos_sensor(OV8858TRULYSensorReg[i].Addr);
		spin_lock(&ov8858trulymipiraw_drv_lock);
		OV8858TRULYSensorReg[i].Para = temp_data;
		spin_unlock(&ov8858trulymipiraw_drv_lock);
    }
}

kal_int32  OV8858TRULY_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV8858TRULY_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void OV8858TRULY_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= OV8858TRULYSensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/ov8865.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= OV8858TRULY_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= OV8858TRULY_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool OV8858TRULY_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * ov8865.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&ov8858trulymipiraw_drv_lock);
          OV8858TRULYSensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&ov8858trulymipiraw_drv_lock);
          OV8858TRULY_write_cmos_sensor(OV8858TRULYSensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&ov8858trulymipiraw_drv_lock);
                    OV8858TRULY_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&ov8858trulymipiraw_drv_lock);
                    break;
                case 1:
                    OV8858TRULY_write_cmos_sensor(OV8858TRULY_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}
#endif

void OV8858TRULY_1224pSetting(void)
{

	/*   Preview of ov8858truly setting                                 */
	/*   @@5.1.2.2 Raw 10bit 1632x1224 30fps 2lane 720M bps/lane   */
	/*   ;Pclk 72MHz                                               */
	/*   ;pixels per line=1928(0x788)                              */
	/*   ;lines per frame=1244(0x4dc)                              */
	OV8858TRULY_write_cmos_sensor(0x0100, 0x00); //
	OV8858TRULY_write_cmos_sensor(0x030e, 0x02); // ; pll2_rdiv //0x00
	OV8858TRULY_write_cmos_sensor(0x030f, 0x09); // ; pll2_divsp
	OV8858TRULY_write_cmos_sensor(0x0312, 0x03); // ; pll2_pre_div0, pll2_r_divdac 01
	OV8858TRULY_write_cmos_sensor(0x3015, 0x00); //01
	OV8858TRULY_write_cmos_sensor(0x3501, 0x4d); // ; exposure M
	OV8858TRULY_write_cmos_sensor(0x3502, 0x40); // ; exposure L
	OV8858TRULY_write_cmos_sensor(0x3508, 0x04); // ; gain H
	OV8858TRULY_write_cmos_sensor(0x3706, 0x35); //
	OV8858TRULY_write_cmos_sensor(0x370a, 0x00); //
	OV8858TRULY_write_cmos_sensor(0x370b, 0xb5); //
	OV8858TRULY_write_cmos_sensor(0x3778, 0x1b); //
	OV8858TRULY_write_cmos_sensor(0x3808, 0x06); // ; x output size H 1632 
	OV8858TRULY_write_cmos_sensor(0x3809, 0x60); // ; x output size L
	OV8858TRULY_write_cmos_sensor(0x380a, 0x04); // ; y output size H 1224
	OV8858TRULY_write_cmos_sensor(0x380b, 0xc8); // ; y output size L
	OV8858TRULY_write_cmos_sensor(0x380c, 0x07); // ; HTS H
	OV8858TRULY_write_cmos_sensor(0x380d, 0x88); // ; HTS L
	OV8858TRULY_write_cmos_sensor(0x380e, 0x04); // ; VTS H
	OV8858TRULY_write_cmos_sensor(0x380f, 0xdc); // ; VTS L
	OV8858TRULY_write_cmos_sensor(0x3814, 0x03); // ; x odd inc
	OV8858TRULY_write_cmos_sensor(0x3821, 0x67); // ; mirror on, bin on
	OV8858TRULY_write_cmos_sensor(0x382a, 0x03); // ; y odd inc
	OV8858TRULY_write_cmos_sensor(0x3830, 0x08); //
	OV8858TRULY_write_cmos_sensor(0x3836, 0x02); //
	OV8858TRULY_write_cmos_sensor(0x3f0a, 0x80); //
	OV8858TRULY_write_cmos_sensor(0x4001, 0x10); // ; total 128 black column
	OV8858TRULY_write_cmos_sensor(0x4022, 0x04); // ; Anchor left end H
	OV8858TRULY_write_cmos_sensor(0x4023, 0xb9); // ; Anchor left end L
	OV8858TRULY_write_cmos_sensor(0x4024, 0x05); // ; Anchor right start H
	OV8858TRULY_write_cmos_sensor(0x4025, 0x2a); // ; Anchor right start L
	OV8858TRULY_write_cmos_sensor(0x4026, 0x05); // ; Anchor right end H
	OV8858TRULY_write_cmos_sensor(0x4027, 0x2b); // ; Anchor right end L
	OV8858TRULY_write_cmos_sensor(0x402b, 0x04); // ; top black line number
	OV8858TRULY_write_cmos_sensor(0x402e, 0x08); // ; bottom black line start
	OV8858TRULY_write_cmos_sensor(0x4500, 0x38); //
	OV8858TRULY_write_cmos_sensor(0x4600, 0x00); //
	OV8858TRULY_write_cmos_sensor(0x4601, 0xcb); //
	OV8858TRULY_write_cmos_sensor(0x382d, 0x7f); //
	OV8858TRULY_write_cmos_sensor(0x0100, 0x01); //
}

void OV8858TRULYPreviewSetting(void)
{
/*          @@5.1.1.2 Raw 10bit 1632x1224 30fps 4lane 720M bps/lane           */
/*          ;;                                                                */
/*          ;; MIPI=720Mbps, SysClk=72Mhz,Dac Clock=360Mhz.                   */
/*          ;;                                                                */
/*          ;Pclk 72MHz                                                       */
/*          ;pixels per line=1928(0x788)                                      */
/*          ;lines per frame=1244(0x4dc)                                      */
/*100 99 1632 1224 ; Resolution                                               */
/*102 80 1                                                                    */


	OV8858TRULY_write_cmos_sensor( 0x0100, 0x00);  //
	mdelay(40);
	OV8858TRULY_write_cmos_sensor( 0x030f, 0x09);  // ; pll2_divsp
	OV8858TRULY_write_cmos_sensor( 0x3501, 0x4d);  // ; exposure M
	OV8858TRULY_write_cmos_sensor( 0x3502, 0x40);  // ; exposure L
	OV8858TRULY_write_cmos_sensor( 0x3508, 0x04);  // ; gain H
                                                                           
OV8858TRULY_write_cmos_sensor(0x030e,0x00);//; pll2_rdiv                                                                     
OV8858TRULY_write_cmos_sensor(0x030f,0x09);//; pll2_divsp                                                                    
OV8858TRULY_write_cmos_sensor(0x0312,0x01);//; pll2_pre_div0, pll2_r_divdac                                                  
OV8858TRULY_write_cmos_sensor(0x3015,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3501,0x4d);//; exposure M                                                                    
OV8858TRULY_write_cmos_sensor(0x3502,0x40);//; exposure L                                                                    
OV8858TRULY_write_cmos_sensor(0x3508,0x04);//; gain H                                                                        
OV8858TRULY_write_cmos_sensor(0x3706,0x35);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370a,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370b,0xb5);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3778,0x1b);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3808,0x06);//; x output size H                                                               
OV8858TRULY_write_cmos_sensor(0x3809,0x60);//; x output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380a,0x04);//; y output size H                                                               
OV8858TRULY_write_cmos_sensor(0x380b,0xc8);//; y output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380c,0x07);//; HTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380d,0x88);//; HTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x380e,0x04);//; VTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380f,0xdc);//; VTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x3814,0x03);//; x odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3821,0x67);//; mirror on, bin on                                                             
OV8858TRULY_write_cmos_sensor(0x382a,0x03);//; y odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3830,0x08);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3836,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3f0a,0x80);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4001,0x10);//; total 128 black column                                                        
OV8858TRULY_write_cmos_sensor(0x4022,0x04);//; Anchor left end H                                                             
OV8858TRULY_write_cmos_sensor(0x4023,0xb9);//; Anchor left end L                                                             
OV8858TRULY_write_cmos_sensor(0x4024,0x05);//; Anchor right start H                                                          
OV8858TRULY_write_cmos_sensor(0x4025,0x2a);//; Anchor right start L                                                          
OV8858TRULY_write_cmos_sensor(0x4026,0x05);//; Anchor right end H                                                            
OV8858TRULY_write_cmos_sensor(0x4027,0x2b);//; Anchor right end L                                                            
OV8858TRULY_write_cmos_sensor(0x402b,0x04);//; top black line number                                                         
OV8858TRULY_write_cmos_sensor(0x402e,0x08);//; bottom black line start                                                       
OV8858TRULY_write_cmos_sensor(0x4500,0x38);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4600,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4601,0xcb);//                                                                                
OV8858TRULY_write_cmos_sensor(0x382d,0x7f);//                                                                                
	mdelay(40);                      
	OV8858TRULY_write_cmos_sensor( 0x0100,0x01);  //



}

void OV8858TRULYCaptureSetting(void)
{
/*           //5.1.2.3 Raw 10bit 3264x2448 15fps 2lane 720M bps/lane      
//;; MIPI=720Mbps, SysClk=72Mhz,Dac Clock=360Mhz.             */               
/*             ;Pclk 144MHz                                                                                           */                 
/*             ;pixels per line=1940(0x794)                                                                    */                    
/*             ;lines per frame=2474(0x9aa)                                                                  */                     
/*100 99 3264 2448 ; Resolution
102 80 1
*/

OV8858TRULY_write_cmos_sensor(0x0100,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x030e,0x02);//; pll2_rdiv                                                                     
OV8858TRULY_write_cmos_sensor(0x030f,0x04);//; pll2_divsp                                                                    
OV8858TRULY_write_cmos_sensor(0x0312,0x03);//; pll2_pre_div0, pll2_r_divdac                                                  
OV8858TRULY_write_cmos_sensor(0x3015,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3501,0x9a);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3502,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3508,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3706,0x6a);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370a,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370b,0x6a);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3778,0x32);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3808,0x0c);//; x output size H                                                               
OV8858TRULY_write_cmos_sensor(0x3809,0xc0);//; x output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380a,0x09);//; y output size H                                                               
OV8858TRULY_write_cmos_sensor(0x380b,0x90);//; y output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380c,0x07);//; HTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380d,0x94);//; HTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x380e,0x09);//; VTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380f,0xaa);//; VTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x3814,0x01);//; x odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3821,0x46);//; mirror on, bin off                                                            
OV8858TRULY_write_cmos_sensor(0x382a,0x01);//; y odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3830,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3836,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3f0a,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4001,0x00);//; total 256 black column                                                        
OV8858TRULY_write_cmos_sensor(0x4022,0x0b);//; Anchor left end H                                                             
OV8858TRULY_write_cmos_sensor(0x4023,0xc3);//; Anchor left end L                                                             
OV8858TRULY_write_cmos_sensor(0x4024,0x0c);//; Anchor right start H                                                          
OV8858TRULY_write_cmos_sensor(0x4025,0x36);//; Anchor right start L                                                          
OV8858TRULY_write_cmos_sensor(0x4026,0x0c);//; Anchor right end H                                                            
OV8858TRULY_write_cmos_sensor(0x4027,0x37);//; Anchor right end L                                                            
OV8858TRULY_write_cmos_sensor(0x402b,0x08);//; top black line number                                                         
OV8858TRULY_write_cmos_sensor(0x402e,0x0c);//; bottom black line start                                                       
OV8858TRULY_write_cmos_sensor(0x4500,0x58);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4600,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4601,0x97);//                                                                                
OV8858TRULY_write_cmos_sensor(0x382d,0xff);//                                                                                
OV8858TRULY_write_cmos_sensor(0x0100,0x01);//



	
}

void OV8858TRULYVideoSetting(void)
{
/*          ;;                                                                */
/*          ;; MIPI=720Mbps, SysClk=72Mhz,Dac Clock=360Mhz.                   */
/*          ;;                                                                */
/*          ;Pclk 72MHz                                                       */
/*          ;pixels per line=1928(0x788)                                      */
/*          ;lines per frame=1244(0x4dc)                                      */
/*100 99 1632 1224 ; Resolution                                               */
/*102 80 1                                                                    */

//5.1.2.2 Raw 10bit 1632x1224 30fps 2lane 720M bps/lane                                      
////;; MIPI=720Mbps, SysClk=144Mhz,Dac Clock=360Mhz. 

	OV8858TRULY_write_cmos_sensor( 0x0100, 0x00);  //
	mdelay(40);
	OV8858TRULY_write_cmos_sensor( 0x030f, 0x09);  // ; pll2_divsp
	OV8858TRULY_write_cmos_sensor( 0x3501, 0x4d);  // ; exposure M
	OV8858TRULY_write_cmos_sensor( 0x3502, 0x40);  // ; exposure L
	OV8858TRULY_write_cmos_sensor( 0x3508, 0x04);  // ; gain H
                                                                           
OV8858TRULY_write_cmos_sensor(0x030e,0x00);//; pll2_rdiv                                                                     
OV8858TRULY_write_cmos_sensor(0x030f,0x09);//; pll2_divsp                                                                    
OV8858TRULY_write_cmos_sensor(0x0312,0x01);//; pll2_pre_div0, pll2_r_divdac                                                  
OV8858TRULY_write_cmos_sensor(0x3015,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3501,0x4d);//; exposure M                                                                    
OV8858TRULY_write_cmos_sensor(0x3502,0x40);//; exposure L                                                                    
OV8858TRULY_write_cmos_sensor(0x3508,0x04);//; gain H                                                                        
OV8858TRULY_write_cmos_sensor(0x3706,0x35);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370a,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370b,0xb5);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3778,0x1b);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3808,0x06);//; x output size H                                                               
OV8858TRULY_write_cmos_sensor(0x3809,0x60);//; x output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380a,0x04);//; y output size H                                                               
OV8858TRULY_write_cmos_sensor(0x380b,0xc8);//; y output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380c,0x07);//; HTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380d,0x88);//; HTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x380e,0x04);//; VTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380f,0xdc);//; VTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x3814,0x03);//; x odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3821,0x67);//; mirror on, bin on                                                             
OV8858TRULY_write_cmos_sensor(0x382a,0x03);//; y odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3830,0x08);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3836,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3f0a,0x80);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4001,0x10);//; total 128 black column                                                        
OV8858TRULY_write_cmos_sensor(0x4022,0x04);//; Anchor left end H                                                             
OV8858TRULY_write_cmos_sensor(0x4023,0xb9);//; Anchor left end L                                                             
OV8858TRULY_write_cmos_sensor(0x4024,0x05);//; Anchor right start H                                                          
OV8858TRULY_write_cmos_sensor(0x4025,0x2a);//; Anchor right start L                                                          
OV8858TRULY_write_cmos_sensor(0x4026,0x05);//; Anchor right end H                                                            
OV8858TRULY_write_cmos_sensor(0x4027,0x2b);//; Anchor right end L                                                            
OV8858TRULY_write_cmos_sensor(0x402b,0x04);//; top black line number                                                         
OV8858TRULY_write_cmos_sensor(0x402e,0x08);//; bottom black line start                                                       
OV8858TRULY_write_cmos_sensor(0x4500,0x38);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4600,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4601,0xcb);//                                                                                
OV8858TRULY_write_cmos_sensor(0x382d,0x7f);//                                                                                
	mdelay(40);                      
	OV8858TRULY_write_cmos_sensor( 0x0100,0x01);  //


}

static void OV8858TRULY_Sensor_Init(void)
{
/*          @@5.1.1.1 Initialization (Global Setting)              */         
/*          ;;                                                     */       
/*          ;; MIPI=720Mbps, SysClk=72Mhz,Dac Clock=360Mhz.        */        
/*          ;;                                                     */   
/*          ;;                                                     */     
/*          ;; v00_01_00 (05/29/2013) : initial setting            */     
/*          ;;                                                     */       

	OV8858TRULY_write_cmos_sensor( 0x0103, 0x01);   // ; software reset
	mdelay(40);
	OV8858TRULY_write_cmos_sensor( 0x0100, 0x00);   // ; software standby
	OV8858TRULY_write_cmos_sensor( 0x0100, 0x00);   // ;
	OV8858TRULY_write_cmos_sensor( 0x0100, 0x00);   // ;
	OV8858TRULY_write_cmos_sensor( 0x0100, 0x00);   // ;
	mdelay(5);
	//OV8858TRULY_write_cmos_sensor( 0x0302, 0x1c);   // ; pll1_multi for 92
	//OV8858TRULY_write_cmos_sensor( 0x0302, 0x1c);   // ; pll1_multi for 82
	//just test  ; modify to 648Mbps/lane
OV8858TRULY_write_cmos_sensor(0x0302,0x1f);  // ; modify to 648Mbps/lane     //0x1e   modify mipi data rate 720Mbps->744Mbps                           
                                                                 
OV8858TRULY_write_cmos_sensor(0x0303,0x00);//; pll1_divm                                                                     
OV8858TRULY_write_cmos_sensor(0x0304,0x03);//; pll1_div_mipi                                                                 
OV8858TRULY_write_cmos_sensor(0x030e,0x00);//; pll2_rdiv                                                                     
OV8858TRULY_write_cmos_sensor(0x030f,0x09);//; pll2_divsp                                                                    
OV8858TRULY_write_cmos_sensor(0x0312,0x01);//; pll2_pre_div0, pll2_r_divdac                                                  
OV8858TRULY_write_cmos_sensor(0x031e,0x0c);//; pll1_no_lat                                                                   
OV8858TRULY_write_cmos_sensor(0x3600,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3601,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3602,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3603,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3604,0x22);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3605,0x30);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3606,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3607,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3608,0x11);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3609,0x28);//                                                                                
OV8858TRULY_write_cmos_sensor(0x360a,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x360b,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x360c,0xdc);//                                                                                
OV8858TRULY_write_cmos_sensor(0x360d,0x40);//                                                                                
OV8858TRULY_write_cmos_sensor(0x360e,0x0c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x360f,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3610,0x07);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3611,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3612,0x88);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3613,0x80);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3614,0x58);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3615,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3616,0x4a);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3617,0xb0);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3618,0x56);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3619,0x70);//                                                                                
OV8858TRULY_write_cmos_sensor(0x361a,0x99);//                                                                                
OV8858TRULY_write_cmos_sensor(0x361b,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x361c,0x07);//                                                                                
OV8858TRULY_write_cmos_sensor(0x361d,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x361e,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x361f,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3638,0xff);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3633,0x0c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3634,0x0c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3635,0x0c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3636,0x0c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3645,0x13);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3646,0x83);//                                                                                
OV8858TRULY_write_cmos_sensor(0x364a,0x07);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3015,0x01);//;                                                                               
OV8858TRULY_write_cmos_sensor(0x3018,0x32);//; MIPI 2 lane                                                                   
OV8858TRULY_write_cmos_sensor(0x3020,0x93);//; Clock switch output normal, pclk_div =/1                                      
OV8858TRULY_write_cmos_sensor(0x3022,0x01);//; pd_mipi enable when rst_sync                                                  
OV8858TRULY_write_cmos_sensor(0x3031,0x0a);//; MIPI 10-bit mode                                                              
OV8858TRULY_write_cmos_sensor(0x3034,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3106,0x01);//; sclk_div, sclk_pre_div                                                        
OV8858TRULY_write_cmos_sensor(0x3305,0xf1);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3308,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3309,0x28);//                                                                                
OV8858TRULY_write_cmos_sensor(0x330a,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x330b,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x330c,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x330d,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x330e,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x330f,0x40);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3307,0x04);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3500,0x00);//; exposure H                                                                    
OV8858TRULY_write_cmos_sensor(0x3501,0x4d);//; exposure M                                                                    
OV8858TRULY_write_cmos_sensor(0x3502,0x40);//; exposure L                                                                    
OV8858TRULY_write_cmos_sensor(0x3503,0x00);//; gain delay 1 frame, exposure delay 1 frame, real gain                         
OV8858TRULY_write_cmos_sensor(0x3505,0x80);//; gain option                                                                   
OV8858TRULY_write_cmos_sensor(0x3508,0x04);//; gain H                                                                        
OV8858TRULY_write_cmos_sensor(0x3509,0x00);//; gain L                                                                        
OV8858TRULY_write_cmos_sensor(0x350c,0x00);//; short gain H                                                                  
OV8858TRULY_write_cmos_sensor(0x350d,0x80);//; short gain L                                                                  
OV8858TRULY_write_cmos_sensor(0x3510,0x00);//; short exposure H                                                              
OV8858TRULY_write_cmos_sensor(0x3511,0x02);//; short exposure M                                                              
OV8858TRULY_write_cmos_sensor(0x3512,0x00);//; short exposure L                                                              
OV8858TRULY_write_cmos_sensor(0x3700,0x18);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3701,0x0c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3702,0x28);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3703,0x19);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3704,0x14);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3705,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3706,0x35);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3707,0x04);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3708,0x24);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3709,0x33);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370a,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370b,0xb5);//                                                                                
OV8858TRULY_write_cmos_sensor(0x370c,0x04);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3718,0x12);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3719,0x31);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3712,0x42);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3714,0x24);//                                                                                
OV8858TRULY_write_cmos_sensor(0x371e,0x19);//                                                                                
OV8858TRULY_write_cmos_sensor(0x371f,0x40);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3720,0x05);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3721,0x05);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3724,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3725,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3726,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3728,0x05);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3729,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x372a,0x03);//                                                                                
OV8858TRULY_write_cmos_sensor(0x372b,0x53);//                                                                                
OV8858TRULY_write_cmos_sensor(0x372c,0xa3);//                                                                                
OV8858TRULY_write_cmos_sensor(0x372d,0x53);//                                                                                
OV8858TRULY_write_cmos_sensor(0x372e,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x372f,0x10);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3730,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3731,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3732,0x14);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3733,0x10);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3734,0x40);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3736,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x373a,0x05);//                                                                                
OV8858TRULY_write_cmos_sensor(0x373b,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x373c,0x0a);//                                                                                
OV8858TRULY_write_cmos_sensor(0x373e,0x03);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3755,0x10);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3758,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3759,0x4c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x375a,0x06);//                                                                                
OV8858TRULY_write_cmos_sensor(0x375b,0x13);//                                                                                
OV8858TRULY_write_cmos_sensor(0x375c,0x20);//                                                                                
OV8858TRULY_write_cmos_sensor(0x375d,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x375e,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x375f,0x14);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3768,0x22);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3769,0x44);//                                                                                
OV8858TRULY_write_cmos_sensor(0x376a,0x44);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3761,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3762,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3763,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3766,0xff);//                                                                                
OV8858TRULY_write_cmos_sensor(0x376b,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3772,0x23);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3773,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3774,0x16);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3775,0x12);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3776,0x04);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3777,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3778,0x1b);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a0,0x44);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a1,0x3d);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a2,0x3d);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a3,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a4,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a5,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a6,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a7,0x44);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a8,0x4c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37a9,0x4c);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3760,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x376f,0x01);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37aa,0x44);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37ab,0x2e);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37ac,0x2e);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37ad,0x33);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37ae,0x0d);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37af,0x0d);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b0,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b1,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b2,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b3,0x42);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b4,0x42);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b5,0x33);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b6,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b7,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b8,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x37b9,0xff);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3800,0x00);//; x start H                                                                     
OV8858TRULY_write_cmos_sensor(0x3801,0x0c);//; x start L                                                                     
OV8858TRULY_write_cmos_sensor(0x3802,0x00);//; y start H                                                                     
OV8858TRULY_write_cmos_sensor(0x3803,0x0c);//; y start L                                                                     
OV8858TRULY_write_cmos_sensor(0x3804,0x0c);//; x end H                                                                       
OV8858TRULY_write_cmos_sensor(0x3805,0xd3);//; x end L                                                                       
OV8858TRULY_write_cmos_sensor(0x3806,0x09);//; y end H                                                                       
OV8858TRULY_write_cmos_sensor(0x3807,0xa3);//; y end L                                                                       
OV8858TRULY_write_cmos_sensor(0x3808,0x06);//; x output size H                                                               
OV8858TRULY_write_cmos_sensor(0x3809,0x60);//; x output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380a,0x04);//; y output size H                                                               
OV8858TRULY_write_cmos_sensor(0x380b,0xc8);//; y output size L                                                               
OV8858TRULY_write_cmos_sensor(0x380c,0x07);//; 03 ; HTS H                                                                    
OV8858TRULY_write_cmos_sensor(0x380d,0x88);//; c4 ; HTS L                                                                    
OV8858TRULY_write_cmos_sensor(0x380e,0x04);//; VTS H                                                                         
OV8858TRULY_write_cmos_sensor(0x380f,0xdc);//; VTS L                                                                         
OV8858TRULY_write_cmos_sensor(0x3810,0x00);//; ISP x win H                                                                   
OV8858TRULY_write_cmos_sensor(0x3811,0x04);//; ISP x win L                                                                   
OV8858TRULY_write_cmos_sensor(0x3813,0x02);//; ISP y win L                                                                   
OV8858TRULY_write_cmos_sensor(0x3814,0x03);//; x odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x3815,0x01);//; x even inc                                                                    
OV8858TRULY_write_cmos_sensor(0x3820,0x00);//; vflip off                                                                     
OV8858TRULY_write_cmos_sensor(0x3821,0x67);//; mirror on, bin on                                                             
OV8858TRULY_write_cmos_sensor(0x382a,0x03);//; y odd inc                                                                     
OV8858TRULY_write_cmos_sensor(0x382b,0x01);//; y even inc                                                                    
OV8858TRULY_write_cmos_sensor(0x3830,0x08);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3836,0x02);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3837,0x18);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3841,0xff);//; window auto size enable                                                       
OV8858TRULY_write_cmos_sensor(0x3846,0x48);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3d85,0x14);//; OTP power up load data enable, OTP powerr up load setting disable             
OV8858TRULY_write_cmos_sensor(0x3f08,0x08);//                                                                                
OV8858TRULY_write_cmos_sensor(0x3f0a,0x80);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4000,0xf1);//; out_range_trig, format_chg_trig, gain_trig, exp_chg_trig, median filter enable
OV8858TRULY_write_cmos_sensor(0x4001,0x10);//; total 128 black column                                                        
OV8858TRULY_write_cmos_sensor(0x4005,0x10);//; BLC target L                                                                  
OV8858TRULY_write_cmos_sensor(0x4002,0x27);//; value used to limit BLC offset                                                
OV8858TRULY_write_cmos_sensor(0x4009,0x81);//; final BLC offset limitation enable                                            
OV8858TRULY_write_cmos_sensor(0x400b,0x0c);//; DCBLC on, DCBLC manual mode on                                                
OV8858TRULY_write_cmos_sensor(0x401b,0x00);//; zero line R coefficient                                                       
OV8858TRULY_write_cmos_sensor(0x401d,0x00);//; zoro line T coefficient                                                       
OV8858TRULY_write_cmos_sensor(0x4020,0x00);//; Anchor left start H                                                           
OV8858TRULY_write_cmos_sensor(0x4021,0x04);//; Anchor left start L                                                           
OV8858TRULY_write_cmos_sensor(0x4022,0x04);//; Anchor left end H                                                             
OV8858TRULY_write_cmos_sensor(0x4023,0xb9);//; Anchor left end L                                                             
OV8858TRULY_write_cmos_sensor(0x4024,0x05);//; Anchor right start H                                                          
OV8858TRULY_write_cmos_sensor(0x4025,0x2a);//; Anchor right start L                                                          
OV8858TRULY_write_cmos_sensor(0x4026,0x05);//; Anchor right end H                                                            
OV8858TRULY_write_cmos_sensor(0x4027,0x2b);//; Anchor right end L                                                            
OV8858TRULY_write_cmos_sensor(0x4028,0x00);//; top zero line start                                                           
OV8858TRULY_write_cmos_sensor(0x4029,0x02);//; top zero line number                                                          
OV8858TRULY_write_cmos_sensor(0x402a,0x04);//; top black line start                                                          
OV8858TRULY_write_cmos_sensor(0x402b,0x04);//; top black line number                                                         
OV8858TRULY_write_cmos_sensor(0x402c,0x02);//; bottom zero line start                                                        
OV8858TRULY_write_cmos_sensor(0x402d,0x02);//; bottom zoro line number                                                       
OV8858TRULY_write_cmos_sensor(0x402e,0x08);//; bottom black line start                                                       
OV8858TRULY_write_cmos_sensor(0x402f,0x02);//; bottom black line number                                                      
OV8858TRULY_write_cmos_sensor(0x401f,0x00);//; interpolation x disable, interpolation y disable, Anchor one disable          
OV8858TRULY_write_cmos_sensor(0x4034,0x3f);//                                                                                
OV8858TRULY_write_cmos_sensor(0x403d,0x04);//; md_precison_en                                                                
OV8858TRULY_write_cmos_sensor(0x4300,0xff);//; clip max H                                                                    
OV8858TRULY_write_cmos_sensor(0x4301,0x00);//; clip min H                                                                    
OV8858TRULY_write_cmos_sensor(0x4302,0x0f);//; clip min L, clip max L                                                        
OV8858TRULY_write_cmos_sensor(0x4316,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4500,0x38);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4503,0x18);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4600,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4601,0xcb);//                                                                                
OV8858TRULY_write_cmos_sensor(0x481f,0x32);//; clk prepare min           

//csq
OV8858TRULYDB("[csq--]OV8858TRULY 0x4800 enter 0x24:\n ");
OV8858TRULY_write_cmos_sensor(0x4800,0x24);
//OV8858TRULY_write_cmos_sensor(0x4801,0x0f);
OV8858TRULY_write_cmos_sensor(0x4837,0x15);//0x16 modify mipi timing  720Mbps->744Mbps

//OV8858TRULY_write_cmos_sensor(0x4837,0x1a);//; global timing          

OV8858TRULY_write_cmos_sensor(0x4850,0x10);//; lane 1 = 1, lane 0 = 0                                                        
OV8858TRULY_write_cmos_sensor(0x4851,0x32);//; lane 3 = 3, lane 2 = 2                                                        
OV8858TRULY_write_cmos_sensor(0x4b00,0x2a);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4b0d,0x00);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4d00,0x04);//; temperature sensor                                                            
OV8858TRULY_write_cmos_sensor(0x4d01,0x18);//;                                                                               
OV8858TRULY_write_cmos_sensor(0x4d02,0xc3);//;                                                                               
OV8858TRULY_write_cmos_sensor(0x4d03,0xff);//;                                                                               
OV8858TRULY_write_cmos_sensor(0x4d04,0xff);//;                                                                               
OV8858TRULY_write_cmos_sensor(0x4d05,0xff);//; temperature sensor                                                            
OV8858TRULY_write_cmos_sensor(0x5000,0x7e);//; slave AWB gain enable, slave AWB statistics enable, master AWB gain enable,   
OV8858TRULY_write_cmos_sensor(0x5001,0x01);//; BLC on                                                                        
OV8858TRULY_write_cmos_sensor(0x5002,0x08);//; H scale off, WBMATCH select slave sensor's gain, WBMATCH off, OTP_DPC         
OV8858TRULY_write_cmos_sensor(0x5003,0x20);//; DPC_DBC buffer control enable, WB                                             
OV8858TRULY_write_cmos_sensor(0x5046,0x12);//                                                                                
OV8858TRULY_write_cmos_sensor(0x5780,0xfc);//dpc control jason 20140318
OV8858TRULY_write_cmos_sensor(0x5781,0xdf);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5782,0x3f);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5783,0x08);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5784,0x0c);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5786,0x20);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5787,0x40);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5788,0x08);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5789,0x08);//dpc control  
OV8858TRULY_write_cmos_sensor(0x578a,0x02);//dpc control  
OV8858TRULY_write_cmos_sensor(0x578b,0x01);//dpc control 
OV8858TRULY_write_cmos_sensor(0x578c,0x01);//dpc control 
OV8858TRULY_write_cmos_sensor(0x578d,0x0c);//dpc control 
OV8858TRULY_write_cmos_sensor(0x578e,0x02);//dpc control 
OV8858TRULY_write_cmos_sensor(0x578f,0x01);//dpc control 
OV8858TRULY_write_cmos_sensor(0x5790,0x01);//dpc control  
OV8858TRULY_write_cmos_sensor(0x5b00,0x02);//OTP DPC start address 
OV8858TRULY_write_cmos_sensor(0x5b01,0x10);//OTP DPC start address 
OV8858TRULY_write_cmos_sensor(0x5b02,0x03);//OTP DPC end address 
OV8858TRULY_write_cmos_sensor(0x5b03,0xcf);//OTP DPC end address 
OV8858TRULY_write_cmos_sensor(0x5b05,0x6c);//dpc control jason 20140318                                                                             
OV8858TRULY_write_cmos_sensor(0x5901,0x00);//; H skip off, V skip off                                                        
OV8858TRULY_write_cmos_sensor(0x5e00,0x00);//; test pattern off                                                              
OV8858TRULY_write_cmos_sensor(0x5e01,0x41);//; window cut enable                                                             
OV8858TRULY_write_cmos_sensor(0x382d,0x7f);//                                                                                
OV8858TRULY_write_cmos_sensor(0x4825,0x3a);//; lpx_p_min                                                                     
OV8858TRULY_write_cmos_sensor(0x4826,0x40);//; hs_prepare_min                                                                
OV8858TRULY_write_cmos_sensor(0x4808,0x25);//; wake up delay in 1/1024 s                                                     
	mdelay(40);
	OV8858TRULY_write_cmos_sensor( 0x0100, 0x01);   //
	mdelay(50);

}

UINT32 OV8858TRULYOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	OV8858TRULYDB("OV8858TRULY Open enter :\n ");
	OV8858TRULY_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mdelay(2);

	for(i=0;i<2;i++)
	{
		sensor_id = (OV8858TRULY_read_cmos_sensor(0x300B)<<8)|OV8858TRULY_read_cmos_sensor(0x300C);
		OV8858TRULYDB("OV8858TRULY READ ID :%x",sensor_id);
		sensor_id += 1;
		if(sensor_id != OV8858_SENSOR_ID_TRULY)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	
	OV8858TRULY_Sensor_Init();
	mdelay(40);
    OV8858TRULY_Init_Para();
	
	#ifdef OV8858TRULY_DEBUG
		OV8858TRULYDB("[OV8858TRULYOpen] enter and exit."); 
	#endif
	#ifndef ORIGINAL_VERSION  
  	update_otp_wb();
	  update_otp_lenc();
	  #ifdef OV8858TRULY_DEBUG
		  OV8858TRULYDB("[OV8858TRULYOpen OTP] OTP done."); 
	  #endif	

  #endif

    return ERROR_NONE;
}


UINT32 OV8858TRULYGetSensorID(UINT32 *sensorID)
{
    int  retry = 2;
	struct otp_struct current_otp;

	OV8858TRULYDB("OV8858TRULYGetSensorID enter :\n ");
    mdelay(5);

    do {
        *sensorID = (OV8858TRULY_read_cmos_sensor(0x300B)<<8)|OV8858TRULY_read_cmos_sensor(0x300C);
		*sensorID += 1;
        if (*sensorID == OV8858_SENSOR_ID_TRULY)
        	{
        		OV8858TRULYDB("Sensor ID = 0x%04x\n", *sensorID);
            	break;
        	}
        OV8858TRULYDB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != OV8858_SENSOR_ID_TRULY) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	OV8858TRULY_Sensor_Init();
	//mdelay(40);
  	//update_otp_wb();
	if(read_otp_info(&current_otp)){
		*sensorID = 0xFFFFFFFF;
		OV8858TRULYDB("Read Module ID Fail!No valid module info OTP data");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
    OV8858TRULYDB("%s Module ID: 0x%x ", __FUNCTION__,g_ov8858truly_module_id);
	if(g_ov8858truly_module_id != 0x2){ 
        *sensorID = 0xFFFFFFFF;
	OV8858TRULYDB("Module ID error,g_ov8858_module_id= 0x%x\n", g_ov8858_module_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	
    #ifndef ORIGINAL_VERSION
	  isNeedReadOtp = true;
    #endif
    return ERROR_NONE;
}

UINT32 OV8858TRULYClose(void)
{
	#ifdef OV8858TRULY_DEBUG
		OV8858TRULYDB("[OV8858TRULYClose]enter and exit.\n");
	#endif

    return ERROR_NONE;
}

kal_uint32 OV8858TRULY_SET_FrameLength_ByVideoMode(UINT16 Video_TargetFps)
{

    UINT32 frameRate = 0;
	kal_uint32 MIN_FrameLength=0;
	
	if(ov8858truly.OV8858TRULYAutoFlickerMode == KAL_TRUE)
	{
		if (Video_TargetFps==30)
			frameRate= OV8858TRULY_AUTOFLICKER_OFFSET_30;
		else if(Video_TargetFps==15)
			frameRate= OV8858TRULY_AUTOFLICKER_OFFSET_15;
		else
			frameRate=Video_TargetFps*10;
	
		MIN_FrameLength = (ov8858truly.videoPclk*10000)/(OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels)/frameRate*10;
	}
	else
		MIN_FrameLength = (ov8858truly.videoPclk*10000) /(OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels)/Video_TargetFps;

     return MIN_FrameLength;


}



UINT32 OV8858TRULYSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV8858TRULYDB("[OV8858TRULYSetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov8858trulymipiraw_drv_lock);
	OV8858TRULY_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov8858trulymipiraw_drv_lock);
	//u2FrameRate=29;

	if(u2FrameRate==0)
	{
		OV8858TRULYDB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV8858TRULYDB("abmornal frame rate seting,pay attention~\n");

    if(ov8858truly.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {

        MIN_Frame_length = OV8858TRULY_SET_FrameLength_ByVideoMode(u2FrameRate);

		if((MIN_Frame_length <=OV8858TRULY_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV8858TRULY_VIDEO_PERIOD_LINE_NUMS;
			OV8858TRULYDB("[OV8858TRULYSetVideoMode]current fps = %d\n", (ov8858truly.videoPclk*10000)  /(OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS)/OV8858TRULY_VIDEO_PERIOD_LINE_NUMS);
		}
		OV8858TRULYDB("[OV8858TRULYSetVideoMode]current fps (10 base)= %d\n", (ov8858truly.videoPclk*10000)*10/(OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels)/MIN_Frame_length);
		extralines = MIN_Frame_length - OV8858TRULY_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&ov8858trulymipiraw_drv_lock);
		ov8858truly.DummyPixels = 0;//define dummy pixels and lines
		ov8858truly.DummyLines = extralines ;
		spin_unlock(&ov8858trulymipiraw_drv_lock);
		
		OV8858TRULY_SetDummy(0, extralines);
    }
	
	OV8858TRULYDB("[OV8858TRULYSetVideoMode]MIN_Frame_length=%d,ov8858truly.DummyLines=%d,u2FrameRate = %d\n",MIN_Frame_length,ov8858truly.DummyLines, u2FrameRate);

    return KAL_TRUE;
}


UINT32 OV8858TRULYSetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	if(bEnable) {   
		spin_lock(&ov8858trulymipiraw_drv_lock);
		ov8858truly.OV8858TRULYAutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov8858trulymipiraw_drv_lock);
        OV8858TRULYDB("OV8858TRULY Enable Auto flicker\n");
    } else {
    	spin_lock(&ov8858trulymipiraw_drv_lock);
        ov8858truly.OV8858TRULYAutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov8858trulymipiraw_drv_lock);
        OV8858TRULYDB("OV8858TRULY Disable Auto flicker\n");
    }

    return ERROR_NONE;
}


UINT32 OV8858TRULYSetTestPatternMode(kal_bool bEnable)
{
    OV8858TRULYDB("[OV8858TRULYSetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable == KAL_TRUE)
    {
        OV8858TRULY_During_testpattern = KAL_TRUE;
		OV8858TRULY_write_cmos_sensor(0x5E00,0x80);
             OV8858TRULY_write_cmos_sensor(0x5000,0x7e);
	OV8858TRULY_write_cmos_sensor(0x5032, 0x04);
	OV8858TRULY_write_cmos_sensor(0x5033, 0x00);
	OV8858TRULY_write_cmos_sensor(0x5034, 0x04);
	OV8858TRULY_write_cmos_sensor(0x5035, 0x00);
	OV8858TRULY_write_cmos_sensor(0x5036, 0x04);
	OV8858TRULY_write_cmos_sensor(0x5037, 0x00);
    }
	else
	{
        OV8858TRULY_During_testpattern = KAL_FALSE;
		OV8858TRULY_write_cmos_sensor(0x5E00,0x00);
             OV8858TRULY_write_cmos_sensor(0x5000,0xfe);
	}

    return ERROR_NONE;
}


/*************************************************************************
*
* DESCRIPTION:
* INTERFACE FUNCTION, FOR USER TO SET MAX  FRAMERATE;
* 
*************************************************************************/
UINT32 OV8858TRULYMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV8858TRULYDB("OV8858TRULYMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV8858TRULY_PREVIEW_PCLK;
			lineLength = OV8858TRULY_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858TRULY_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8858trulymipiraw_drv_lock);
			ov8858truly.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&ov8858trulymipiraw_drv_lock);
			OV8858TRULY_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV8858TRULY_VIDEO_PCLK;
			lineLength = OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858TRULY_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8858trulymipiraw_drv_lock);
			ov8858truly.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&ov8858trulymipiraw_drv_lock);
			OV8858TRULY_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV8858TRULY_CAPTURE_PCLK;
			lineLength = OV8858TRULY_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858TRULY_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8858trulymipiraw_drv_lock);
			ov8858truly.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&ov8858trulymipiraw_drv_lock);
			OV8858TRULY_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;

}


UINT32 OV8858TRULYMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = OV8858TRULY_MAX_FPS_PREVIEW;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = OV8858TRULY_MAX_FPS_CAPTURE;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = OV8858TRULY_MAX_FPS_CAPTURE;
			break;		
		default:
			break;
	}

	return ERROR_NONE;

}


void OV8858TRULY_NightMode(kal_bool bEnable)
{
	
	#ifdef OV8858TRULY_DEBUG
	OV8858TRULYDB("[OV8858TRULY_NightMode]enter and exit.\n");
	#endif
}


#if 0
#endif
UINT32 OV8858TRULYPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV8858TRULYDB("OV8858TRULYPreview enter:");

	OV8858TRULYPreviewSetting();

	spin_lock(&ov8858trulymipiraw_drv_lock);
	ov8858truly.sensorMode = SENSOR_MODE_PREVIEW; 
	ov8858truly.DummyPixels = 0;
	ov8858truly.DummyLines = 0 ;
	OV8858TRULY_FeatureControl_PERIOD_PixelNum=OV8858TRULY_PV_PERIOD_PIXEL_NUMS+ ov8858truly.DummyPixels;
	OV8858TRULY_FeatureControl_PERIOD_LineNum=OV8858TRULY_PV_PERIOD_LINE_NUMS+ov8858truly.DummyLines;
	//TODO~
	//ov8858truly.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov8858trulymipiraw_drv_lock);
	
	//TODO~
    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV8858TRULYDB("OV8858TRULYPreview exit:\n");

	  
    return ERROR_NONE;
}


UINT32 OV8858TRULYVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV8858TRULYDB("OV8858TRULYVideo enter:");

	OV8858TRULYVideoSetting();

	spin_lock(&ov8858trulymipiraw_drv_lock);
	ov8858truly.sensorMode = SENSOR_MODE_VIDEO;
	OV8858TRULY_FeatureControl_PERIOD_PixelNum=OV8858TRULY_VIDEO_PERIOD_PIXEL_NUMS+ ov8858truly.DummyPixels;
	OV8858TRULY_FeatureControl_PERIOD_LineNum=OV8858TRULY_VIDEO_PERIOD_LINE_NUMS+ov8858truly.DummyLines;
	ov8858truly.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov8858trulymipiraw_drv_lock);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV8858TRULYDB("OV8865Video exit:\n");
    return ERROR_NONE;
}


UINT32 OV8858TRULYCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = ov8858truly.shutter;
	kal_uint32 temp_data;


	OV8858TRULYDB("OV8858TRULYCapture enter:\n");

	OV8858TRULYCaptureSetting();

	spin_lock(&ov8858trulymipiraw_drv_lock);
	ov8858truly.sensorMode = SENSOR_MODE_CAPTURE;
	//TODO~
	//ov8858truly.imgMirror = sensor_config_data->SensorImageMirror;
	ov8858truly.DummyPixels = 0;
	ov8858truly.DummyLines = 0 ;
	OV8858TRULY_FeatureControl_PERIOD_PixelNum = OV8858TRULY_FULL_PERIOD_PIXEL_NUMS + ov8858truly.DummyPixels;
	OV8858TRULY_FeatureControl_PERIOD_LineNum = OV8858TRULY_FULL_PERIOD_LINE_NUMS + ov8858truly.DummyLines;
	spin_unlock(&ov8858trulymipiraw_drv_lock);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY

	#if 0
	if(OV8858TRULY_During_testpattern == KAL_TRUE)
	{
		//TODO~
		//Test pattern
		OV8858TRULY_write_cmos_sensor(0x5E00,0x80);
	}
	#endif
	OV8858TRULYDB("OV8865Capture exit:\n");
    return ERROR_NONE;

}	

#if 0
#endif

UINT32 OV8858TRULYGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV8858TRULYDB("OV8858TRULYGetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= OV8858TRULY_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV8858TRULY_IMAGE_SENSOR_PV_HEIGHT;
	
    pSensorResolution->SensorFullWidth		= OV8858TRULY_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV8858TRULY_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorVideoWidth		= OV8858TRULY_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV8858TRULY_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   

UINT32 OV8858TRULYGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    OV8858TRULYDB("OV8858TRULYGetInfo enter!!\n");
	spin_lock(&ov8858trulymipiraw_drv_lock);
	ov8858truly.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov8858trulymipiraw_drv_lock);

    pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 3;
    pSensorInfo->VideoDelayFrame = 3;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;	    
    pSensorInfo->AESensorGainDelayFrame = 0;
    pSensorInfo->AEISPGainDelayFrame = 2;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
	pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;


    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858TRULY_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8858TRULY_PV_Y_START;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858TRULY_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV8858TRULY_VIDEO_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;//0,4,14,32,40
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858TRULY_FULL_X_START;	
            pSensorInfo->SensorGrabStartY = OV8858TRULY_FULL_Y_START;	

            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858TRULY_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8858TRULY_PV_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &OV8858TRULYSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    OV8858TRULYDB("OV8858TRULYGetInfo exit!!\n");

    return ERROR_NONE;
}   /* OV8858TRULYGetInfo() */



UINT32 OV8858TRULYControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov8858trulymipiraw_drv_lock);
		OV8858TRULYCurrentScenarioId = ScenarioId;
		spin_unlock(&ov8858trulymipiraw_drv_lock);
		
		OV8858TRULYDB("[OV8858TRULYControl]OV8858TRULYCurrentScenarioId=%d\n",OV8858TRULYCurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV8858TRULYPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV8858TRULYVideo(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV8858TRULYCapture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* OV8858TRULYControl() */


UINT32 OV8858TRULYFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV8858TRULY_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV8858TRULY_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV8858TRULY_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV8858TRULY_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV8858TRULYCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV8858TRULY_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV8858TRULY_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV8858TRULY_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV8858TRULY_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV8858TRULY_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV8858TRULY_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:  
           	OV8858TRULY_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV8858TRULY_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV8858TRULY_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV8858TRULY_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov8858trulymipiraw_drv_lock);
                OV8858TRULYSensorCCT[i].Addr=*pFeatureData32++;
                OV8858TRULYSensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov8858trulymipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8858TRULYSensorCCT[i].Addr;
                *pFeatureData32++=OV8858TRULYSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov8858trulymipiraw_drv_lock);
                OV8858TRULYSensorReg[i].Addr=*pFeatureData32++;
                OV8858TRULYSensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov8858trulymipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8858TRULYSensorReg[i].Addr;
                *pFeatureData32++=OV8858TRULYSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV8858_SENSOR_ID_TRULY;
                memcpy(pSensorDefaultData->SensorEngReg, OV8858TRULYSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV8858TRULYSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV8858TRULYSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV8858TRULY_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV8858TRULY_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV8858TRULY_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV8858TRULY_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV8858TRULY_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV8858TRULY_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
			//TODO~
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
			OV8858TRULYSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV8858TRULYGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			//TODO~
			OV8858TRULYSetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV8858TRULYMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV8858TRULYMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			//TODO~
			OV8858TRULYSetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
			*pFeatureReturnPara32=OV8858TRULY_TEST_PATTERN_CHECKSUM; 		  
			*pFeatureParaLen=4; 							
		     break;
        default:
            break;
    }
    return ERROR_NONE;
}	


SENSOR_FUNCTION_STRUCT	SensorFuncOV8858TRULY=
{
    OV8858TRULYOpen,
    OV8858TRULYGetInfo,
    OV8858TRULYGetResolution,
    OV8858TRULYFeatureControl,
    OV8858TRULYControl,
    OV8858TRULYClose
};

UINT32 OV8858TRULY_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV8858TRULY;

    return ERROR_NONE;
}  

