/*
 Code based on:
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.

 Modified to work only with Dynamixel XL-320 actuator.

 Modifications made by Luis G III for HelloSpoon robot.
 Webpage: http://hellospoonrobot.com
 Twitter: @HelloSpoon
 Youtube: http://youtube.com/user/hellospoonrobot

 This file can be used and be modified by anyone, 
 don't forget to say thank you to OP!
 
 */

#ifndef XL320_H_
#define XL320_H_

/*EEPROM Area*/
#define XL_MODEL_NUMBER_L           0
#define XL_MODEL_NUMBER_H           1
#define XL_VERSION                  2
#define XL_ID                       3
#define XL_BAUD_RATE                4
#define XL_RETURN_DELAY_TIME        5
#define XL_CW_ANGLE_LIMIT_L         6
#define XL_CW_ANGLE_LIMIT_H         7
#define XL_CCW_ANGLE_LIMIT_L        8
#define XL_CCW_ANGLE_LIMIT_H        9
#define XL_CONTROL_MODE             11
#define XL_LIMIT_TEMPERATURE        12
#define XL_DOWN_LIMIT_VOLTAGE       13
#define XL_UP_LIMIT_VOLTAGE         14
#define XL_MAX_TORQUE_L             15
#define XL_MAX_TORQUE_H             16
#define XL_RETURN_LEVEL             17
#define XL_ALARM_SHUTDOWN           18
/*RAM Area*/
#define XL_TORQUE_ENABLE            24
#define XL_LED                      25
#define XL_D_GAIN    				27
#define XL_I_GAIN      				28
#define XL_P_GAIN    				29
#define XL_GOAL_POSITION_L          30
#define XL_GOAL_SPEED_L             32
#define XL_GOAL_TORQUE 		        35
#define XL_PRESENT_POSITION         37
#define XL_PRESENT_SPEED            39
#define XL_PRESENT_LOAD             41
#define XL_PRESENT_VOLTAGE          45
#define XL_PRESENT_TEMPERATURE      46
#define XL_REGISTERED_INSTRUCTION   47
#define XL_MOVING                   49
#define XL_HARDWARE_ERROR           50
#define XL_PUNCH                    51

#define Tx_MODE                     1
#define Rx_MODE                     0

#include <inttypes.h>
#include <Stream.h>

class XL320 {
private:
	unsigned char Direction_Pin;
	volatile char gbpParamEx[130+10];
	Stream *stream;

  void nDelay(uint32_t nTime);


public:
	XL320(); 
	virtual ~XL320();	
	
	void begin(Stream &stream);
	
	void moveJoint(int id, int value);
	void setJointSpeed(int id, int value);
	void LED(int id, char led_color[]);
	void setJointTorque(int id, int value);

	void TorqueON(int id);
	void TorqueOFF(int id);

	void quickTest();

	int getSpoonLoad();
	int getJointPosition(int id);
	int getJointSpeed(int id);
	int getJointLoad(int id);
	int getJointTemperature(int id);
	int isJointMoving(int id);

	int sendPacket(int id, int Address, int value);
	int readPacket(unsigned char *buffer, size_t size);

	int RXsendPacket(int id, int Address);
	int RXsendPacket(int id, int Address, int size);

	int flush();

	class Packet {
	  bool freeData;
	  public:
	    unsigned char *data;
	    size_t data_size;

	    // wrap a received data stream in an Packet object for analysis
	    Packet(unsigned char *data, size_t size);
	    // build a packet into the pre-allocated data array
	    // if data is null it will be malloc'ed and free'd on destruction.
	    
	    Packet(
	      unsigned char *data, 
	      size_t        size,
	      unsigned char id,
	      unsigned char instruction,
	      int           parameter_data_size,
	      ...);
	    ~Packet();
	    unsigned char getId();
	    int getLength();
	    int getSize();
	    int getParameterCount();
	    unsigned char getInstruction();
            unsigned char getParameter(int n);
	    bool isValid();

	    void toStream(Stream &stream);

	};
};

#endif
