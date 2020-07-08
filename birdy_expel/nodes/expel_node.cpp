/***** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******source file****
* File name          : expel_node.cpp
* Author             : Jinliang Yang
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2020/07/06          Jinliang Yang       Created
*******************************************************************************/
#include "expel.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "agv_charger_node");

    Expel expel;

    ros::Rate loop_rate(200); 
    while(ros::ok()) { 
        expel.parsingRecvData();
        expel.lowTask();

        ros::spinOnce(); 
        loop_rate.sleep(); 
    }

    return 0;
};

/******** (C) Copyright, Shenzhen SUNWIN Intelligent Co.,Ltd. ******** End *****/