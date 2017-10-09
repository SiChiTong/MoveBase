#include "VL6180x.h"
#include "string.h"
#include "iic.h"
#include "usart.h"
#include "fallen.h"
#include "delay.h"
//#include "global.h"


unsigned char LaserOffset = 0 ;

struct sensor
{
	int16_t light[10];					//¼¤¹â
	uint8_t ultra[7];						//³¬Éù²¨
	uint8_t pyr_state;					//ÈÈÊÍµç
	uint8_t motor_power_state;	//µç»úÉÏµç×´Ì¬
}allsensor = {{400,400,400,400,400,400,400,400,400,400},{0,0,0,0,0,0,0},0x00,0x01};


uint8_t stop_range = 45; 				//ÖÆ¶¯¾àÀë

uint8_t time_count = 0;

uint8_t count_copy = 0;

uint8_t status = 0;

uint8_t sensor_status[5] = {0};


static	int alpha =(int)(0.1*(1<<16));    /* range distance running average cofs */
void init_vl6180x_I2C0(unsigned char DeviceAddr)
{
		  /* USER CODE BEGIN 1 */
		single_write_I2C0( DeviceAddr, 0x0207, 0x01);
    single_write_I2C0( DeviceAddr, 0x0208, 0x01);
    single_write_I2C0( DeviceAddr, 0x0096, 0x00);
    single_write_I2C0( DeviceAddr, 0x0097, 0x54);
    single_write_I2C0( DeviceAddr, 0x00e3, 0x00);
    single_write_I2C0( DeviceAddr, 0x00e4, 0x04);
    single_write_I2C0( DeviceAddr, 0x00e5, 0x02);
    single_write_I2C0( DeviceAddr, 0x00e6, 0x01);
    single_write_I2C0( DeviceAddr, 0x00e7, 0x03);
    single_write_I2C0( DeviceAddr, 0x00f5, 0x02);
    single_write_I2C0( DeviceAddr, 0x00d9, 0x05);

		multi_write_I2C0(	DeviceAddr,0x00da,0x00CF03F7);

    single_write_I2C0( DeviceAddr, 0x009f, 0x00);
    single_write_I2C0( DeviceAddr, 0x00a3, 0x28);
    single_write_I2C0( DeviceAddr, 0x00b7, 0x00);
    single_write_I2C0( DeviceAddr, 0x00bb, 0x28);
    single_write_I2C0( DeviceAddr, 0x00b2, 0x09);
    single_write_I2C0( DeviceAddr, 0x00ca, 0x09);
    single_write_I2C0( DeviceAddr, 0x0198, 0x01);
    single_write_I2C0( DeviceAddr, 0x01b0, 0x17);
    single_write_I2C0( DeviceAddr, 0x01ad, 0x00);
    single_write_I2C0( DeviceAddr, 0x00ff, 0x05);
    single_write_I2C0( DeviceAddr, 0x0100, 0x05);
    single_write_I2C0( DeviceAddr, 0x0199, 0x05);
    single_write_I2C0( DeviceAddr, 0x01a6, 0x1b);
    single_write_I2C0( DeviceAddr, 0x01ac, 0x3e);
    single_write_I2C0( DeviceAddr, 0x01a7, 0x1f);
    single_write_I2C0( DeviceAddr, 0x0030, 0x00);
    single_write_I2C0( DeviceAddr, 0x0011, 0x10);
    single_write_I2C0( DeviceAddr, 0x010a, 0x30);
    single_write_I2C0( DeviceAddr, 0x003f, 0x46);
    single_write_I2C0( DeviceAddr, 0x0031, 0xFF);
    single_write_I2C0( DeviceAddr, 0x0040, 0x63);
    single_write_I2C0( DeviceAddr, 0x002e, 0x01);
    single_write_I2C0( DeviceAddr, 0x002c, 0xff);
    single_write_I2C0( DeviceAddr, 0x001b, 0x09);
    single_write_I2C0( DeviceAddr, 0x003e, 0x31);
    single_write_I2C0( DeviceAddr, 0x0014, 0x24);

    single_write_I2C0(	DeviceAddr, 0x001C, 0x32);
    double_write_I2C0(	DeviceAddr, 0x0022, 0x0060);
    double_write_I2C0(	DeviceAddr, 0x0096, 0x00fd);
		single_write_I2C0( DeviceAddr, 0x024, 0x08);
		
		single_write_I2C0( DeviceAddr, 0x016, 0x00);


		single_write_I2C0( DeviceAddr, 0x03f, 0x46);
		single_write_I2C0( DeviceAddr, 0x019, 0xc8);
		single_write_I2C0( DeviceAddr, 0x01A, 0x0a);
    double_write_I2C0( DeviceAddr, 0x0040, 0x0063);
    single_write_I2C0( DeviceAddr, 0x003e, 0x14);
    single_write_I2C0( DeviceAddr, 0x003f, 0x40);
    single_write_I2C0( DeviceAddr, 0x003c, 0x00);
    single_write_I2C0( DeviceAddr, 0x003A, 0xff);
		single_write_I2C0( DeviceAddr, 0x015, 0x07);
		single_write_I2C0( DeviceAddr, 0x01AC, 0x3c);
		single_write_I2C0( DeviceAddr, 0x0f2, 0x05);
}


unsigned char GetFallenData(unsigned char DeviceAddr)
{
		int status;
		int new_switch_state;
		int switch_state = -1;
		unsigned char pdata = 0;
		unsigned char pdata1 = 0;
		unsigned char pdata_3 = 0;
		
		unsigned short range = 0; 
		int alpha =(int)(0.1*(1<<16));    /* range distance running average cofs */
	
			status = single_read_I2C0(DeviceAddr,0x000,&pdata1);
			if(pdata1 == 0xB4)
			{
				single_write_I2C0(DeviceAddr, 0x015, 0x01);							//???D??
				single_read_I2C0(DeviceAddr,RESULT__RANGE_STATUS,&pdata); //µè'y?aê?
				OSTimeDly(1);
				if(pdata==0)
				{
//					single_read_I2C0(DeviceAddr,RESULT__RANGE_STATUS,&pdata);
					range = DEV_ERR;
					return range;
				}
//				else
//				{
//					range = DATA_ERR;
//					return range;
//				}
				single_write_I2C0(DeviceAddr, 0x018, 0x01);		//?aê?2aá?
				OSTimeDly(4);
				status = single_read_I2C0(DeviceAddr,0x000,&pdata1);
				if(pdata1 == 0xB4)
				{
					single_read_I2C0(DeviceAddr,RESULT__INTERRUPT_STATUS_GPIO,&pdata);
					if(pdata == 4)
					{
						single_read_I2C0(DeviceAddr,RESULT__RANGE_VAL,&pdata1);
						if(pdata1 != 255)
						{
							pdata_3 = pdata1;
							range = pdata1;
							range = range + LaserOffset;
						}
						else
						{
							range = DATA_ERR;
						}
					}
					else
					{
						range=DEV_ERR;
					}
				}
				else
				{
					range = DEV_ERR;
				}
			}
			else
			{
				range = DEV_ERR ;
			}
			return range;
}


#if 1
void Vl6180X_Init(void)
{
	VL6180X_CE1_OPEN;
	VL6180X_CE2_CLOSE;
	VL6180X_CE3_CLOSE;
//	VL6180X_CE4_CLOSE;
//	VL6180X_CE5_CLOSE;
//	VL6180X_CE6_CLOSE;
	
	OSTimeDly(1);
	single_write_I2C0(0x52, 0x0212, SLAVE_ADDRESS1);
	OSTimeDly(2);
	
	VL6180X_CE2_OPEN;
	OSTimeDly(1);
	single_write_I2C0(0x52, 0x0212, SLAVE_ADDRESS2);
	OSTimeDly(2);
	
	VL6180X_CE3_OPEN;
	OSTimeDly(1);
	single_write_I2C0(0x52, 0x0212, SLAVE_ADDRESS3);
	OSTimeDly(2);
	
//	VL6180X_CE4_OPEN;
//	OSTimeDly(1);
//	single_write_I2C0(0x52, 0x0212, SLAVE_ADDRESS4);
//	OSTimeDly(2);
//	
//	VL6180X_CE5_OPEN;
//	OSTimeDly(1);
//	single_write_I2C0(0x52, 0x0212, SLAVE_ADDRESS5);
//	OSTimeDly(2);
	
	init_vl6180x_I2C0(VL6180X_ADDRESS1);
	OSTimeDly(2);
	init_vl6180x_I2C0(VL6180X_ADDRESS2);
	OSTimeDly(2);
	init_vl6180x_I2C0(VL6180X_ADDRESS3);
	OSTimeDly(2);
//	init_vl6180x_I2C0(VL6180X_ADDRESS4);
//	OSTimeDly(2);
//	init_vl6180x_I2C0(VL6180X_ADDRESS5);
//	OSTimeDly(2);
}
#endif