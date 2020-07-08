/***** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******header file****
* File name          : expel.h
* Author             : Jinliang Yang
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author          	Described
* V1.00     2020/07/6           Jinliang Yang       Created
*******************************************************************************/
#include <ros/ros.h> 
#include <serial/serial.h>  
#include <iostream>  
#include <stdlib.h>

#include "birdy_config/ExpelControl.h"
#include "birdy_config/ExpelData.h"
     
using namespace std;

#define EXPEL_WRITE				(0x15)
#define EXPEL_READ	    		(0x05)	

#define EXPEL_POWER_OFF			(0)
#define EXPEL_POWER_ON			(1<<7)
#define EXPEL_PLAY_DISPEL   	(0)
#define EXPEL_TURN_OFF_DISPEL	(1<<6)
#define EXPEL_NOT_FIRE			(0)
#define EXPEL_SINGLE_FIRE		(1<<4)
#define EXPEL_BURST_FIRE		(2<<4)
#define EXPEL_TURN_LEFT_CANCEL	(0)
#define EXPEL_TURN_LEFT_ENABLE	(1<<3)
#define EXPEL_TURN_RIGHT_CANCEL	(0)
#define EXPEL_TURN_RIGHT_ENABLE	(1<<2)
#define EXPEL_TURN_UP_CANCEL	(0)
#define EXPEL_TURN_UP_ENABLE	(1<<1)
#define EXPEL_TURN_DOWN_CANCEL	(0)
#define EXPEL_TURN_DOWM_ENABLE	(1)

enum RECV_STEP{
	STEP_HEAD = 0,
	STEP_ADDR = 1,
	STEP_LEN = 2,
	STEP_DIR = 3,
	STEP_DATA = 4
};

typedef struct {
	uint8_t pressure;
	uint8_t commamd;
	int16_t current;
	uint16_t voltage;
}RecvData;

typedef struct {
    uint8_t dataCount;
    uint8_t	dataLength;
    uint8_t dataCheckSum;
    uint8_t data[11];
    enum RECV_STEP step = STEP_HEAD;
}ExpelComm;

class Expel
{
public:
	Expel();
	~Expel();

	void parsingRecvData(void);
	void writeCommand(uint8_t cmd);
	void readStatus(void);
	void lowTask(void);
	
private:
	void expelControlCallback(const birdy_config::ExpelControl::ConstPtr &msg);

    ros::NodeHandle _nh; 
    ros::Subscriber _sub_expel_control;
    ros::Publisher _pub_expel_data; 

    serial::Serial _ser;  
    uint8_t buffer[256];

    RecvData _data;
    ExpelComm _comm;

    string _com = "/dev/ttyUSB0";
    int32_t _baud = 115200;
	int32_t _device_id = 0;
};

/******** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******** End *****/