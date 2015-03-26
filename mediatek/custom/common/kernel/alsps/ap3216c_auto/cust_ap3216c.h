

#ifndef __CUST_AP3216C_H__
#define __CUST_AP3216C_H__

/* For ps calibration */
#define CUST_AP3216C_PS_THRES_CLOSE		135	 // Close 
#define CUST_AP3216C_PS_THRES_FAR		55	 // Far away 

/*AP3216C ps cust setting*/

#define CUST_AP3216C_PS_CONFIGERATION  	0x14 // PS Configuration		//2T, PS gain=2, 1 conversion time 
#define CUST_AP3216C_PS_LEDCTRL  		0x13 // PS LED Control 		//default 0x13: 1 pulse, 100%=110mA 
#define CUST_AP3216C_PS_MEAN_TIME 		0x00 // PS meam time 		//default 0x00: mean time=12.5ms 

/*******************************************/

/*AP3216C als cust setting*/
#define CUST_AP3216C_ALS_CONFIGERATION	0x00 //ALS Configuration      //default 0x00: 20661lux, 1 conversion time 

#endif


