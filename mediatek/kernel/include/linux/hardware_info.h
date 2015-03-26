#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#define HARDWARE_MAX_ITEM_LONGTH		64

enum{
	HARDWARE_LCD = 0,
	HARDWARE_TP,
	HARDWARE_FLASH,
	HARDWARE_FRONT_CAM,
	HARDWARE_BACK_CAM,
	HARDWARE_BT,
	HARDWARE_WIFI,
	HARDWARE_ACCELEROMETER,
	HARDWARE_ALSPS,
	HARDWARE_GYROSCOPE,
	HARDWARE_MAGNETOMETER,	
	HARDWARE_GPS,
	HARDWARE_FM,
	HARDWARE_SDSTATUS,
	HARDWARE_BAT,
	HARDWARE_MAX_ITEM
};


#define HARDWARE_ID						'H'
#define HARDWARE_LCD_GET				_IOWR(HARDWARE_ID, 0x01, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_TP_GET					_IOWR(HARDWARE_ID, 0x02, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FLASH_GET				_IOWR(HARDWARE_ID, 0x03, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FRONT_CAM_GET			_IOWR(HARDWARE_ID, 0x04, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BACK_CAM_GET			_IOWR(HARDWARE_ID, 0x05, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_ACCELEROMETER_GET		_IOWR(HARDWARE_ID, 0x06, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_ALSPS_GET			    _IOWR(HARDWARE_ID, 0x07, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_GYROSCOPE_GET			_IOWR(HARDWARE_ID, 0x08, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_MAGNETOMETER_GET		_IOWR(HARDWARE_ID, 0x09, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BT_GET					_IOWR(HARDWARE_ID, 0x10, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_WIFI_GET				_IOWR(HARDWARE_ID, 0x11, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_GPS_GET			    _IOWR(HARDWARE_ID, 0x12, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_FM_GET			        _IOWR(HARDWARE_ID, 0x13, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_SDCARD_STATUS			_IOWR(HARDWARE_ID, 0x14, char[HARDWARE_MAX_ITEM_LONGTH])
#define HARDWARE_BAT_GET				_IOWR(HARDWARE_ID, 0x15, char[HARDWARE_MAX_ITEM_LONGTH])





int hardwareinfo_set_prop(int cmd, char *name);

#endif //__HARDWARE_H__
