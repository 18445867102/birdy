/***** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******source file****
* File name          : expel.cpp
* Author             : Jinliang Yang
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author          	Described
* V1.00     2020/07/6           Jinliang Yang       Created
*******************************************************************************/
#include "expel.h"

Expel::Expel() {    
	_sub_expel_control = _nh.subscribe("/expel_control", 10, &Expel::expelControlCallback, this);
	_pub_expel_data = _nh.advertise<birdy_config::ExpelData>("/expel_data", 10);

	_nh.getParam("com_port",_com);
    _nh.getParam("baud",_baud);
	_nh.getParam("device_id", _device_id);

	// open serial.
	try {    
        _ser.setPort(_com); 
        _ser.setBaudrate(_baud); 
        serial::Timeout to = serial::Timeout::simpleTimeout(50); 
        _ser.setTimeout(to); 
        _ser.open();
    } catch (serial::IOException& e) { 
        ROS_ERROR("%s %d",__FILE__, __LINE__); 
    } 

    writeCommand(EXPEL_POWER_ON);
}

void Expel::expelControlCallback(const birdy_config::ExpelControl::ConstPtr &msg) {
	if (msg->cmd == birdy_config::ExpelControl::START_EXPEL) {
		writeCommand(EXPEL_PLAY_DISPEL | EXPEL_BURST_FIRE);
	} else if (msg->cmd == birdy_config::ExpelControl::STOP_EXPEL) {
		writeCommand(EXPEL_TURN_OFF_DISPEL | EXPEL_NOT_FIRE);
	}
}

void Expel::parsingRecvData(void) {
	static uint16_t noDataCount = 0;
    size_t n = _ser.available();
    if (n == 0) {
        noDataCount++;
        if (noDataCount > 400) {
            //ROS_ERROR("COMMUNICATION FAILED.");
            noDataCount = 0;
        }
        return;
    }   

	noDataCount = 0;
    try {    
        n = _ser.read(buffer, n);
    } catch (serial::IOException& e) { 
        ROS_ERROR("serial read error."); 
    } 
    
    for (int i=0; i<n; i++) {
    	//ROS_INFO("0x%x", buffer[i]);
        switch(_comm.step){
        case STEP_HEAD:
            if (buffer[i] == 0xbb) {
            	_comm.data[0] = 0xbb;
                _comm.step = STEP_ADDR;
            }
            break;
        case STEP_ADDR:
            if (buffer[i] == (uint8_t)_device_id) {
            	_comm.data[1] = (uint8_t)_device_id;
              	_comm.step = STEP_LEN;
            } else {
              	_comm.step = STEP_HEAD;
            }
            break;
        case STEP_LEN:
            if (buffer[i] == 0x0B) {
            	_comm.data[2] = 0x0B;
            	_comm.dataLength = 0x0B;
              	_comm.step = STEP_DIR;
            } else {
              	_comm.step = STEP_HEAD;
            }
            break; 
        case STEP_DIR:
            if (buffer[i] == 0x05) {
            	_comm.data[3] = 0x05;
              	_comm.step = STEP_DATA;
              	_comm.dataCount = 4;
            } else {
              	_comm.step = STEP_HEAD;
            }
            break;
        case STEP_DATA:
        	_comm.data[_comm.dataCount++] = buffer[i];
            if (_comm.dataCount == _comm.dataLength) {
            	_comm.dataCheckSum = 0;
            	for(int i = 0; i < 10; i++)
            	{
            		_comm.dataCheckSum += _comm.data[i];
            	}

            	if (_comm.dataCheckSum == buffer[10]) {
            		_data.pressure = (_comm.data[9]&0x80) >> 7;
            		_data.commamd = _comm.data[4];
            		_data.current = (int16_t)(_comm.data[8]<<8 | _comm.data[7]);
            		_data.voltage = uint16_t(_comm.data[6]<<8 | _comm.data[5]);
            		ROS_INFO("cmd:%d  pressure:%d current:%.3f A voltage:%.3f V",
            			_data.commamd,
            			_data.pressure,
            			(double)(_data.current/1000.0),
            			(double)(_data.voltage/1000.0));
            	} 
            	_comm.step = STEP_HEAD;
            }
        	break;
        default:
            _comm.step = STEP_HEAD;
            break;
      	}
    }
}

void Expel::lowTask(void) {
	static uint64_t count = 0;

	if(count % 200 == 0) {
		readStatus();

		birdy_config::ExpelData d;
		d.pressure = _data.pressure;
		d.current = _data.current;
		d.voltage = _data.voltage;
		_pub_expel_data.publish(d);
	}

	count++;
}

void Expel::writeCommand(uint8_t cmd) {
	uint8_t data[6];

	data[0] = 0xaa;
	data[1] = (uint8_t)_device_id;
	data[2] = 0x06;
	data[3] = 0x15;
	data[4] = cmd;
	data[5] = 0;

	for(int i=0;i<5;i++){
		data[5]+=data[i];
	}

	ROS_INFO("cmd  is:%d", cmd);

    if (_ser.isOpen()) {
        _ser.write(data, 6);
    } else { 
        ROS_ERROR("serial not open.");
    } 
}

void Expel::readStatus(void) {
	uint8_t data[6];

	data[0] = 0xaa;
	data[1] = (uint8_t)_device_id;
	data[2] = 0x06;
	data[3] = 0x05;
	data[4] = 0;
	data[5] = 0;

	for(int i=0;i<5;i++){
		data[5]+=data[i];
	}

    if (_ser.isOpen()) {
        _ser.write(data, 6);
    } else { 
        ROS_ERROR("serial not open.");
    }
}

Expel::~Expel() {

}

/******** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******** End *****/