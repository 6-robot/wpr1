#include "SerialCom.h"
#pragma once

class CWPR1_driver : public CSerialCom
{
public:
    CWPR1_driver();
    ~CWPR1_driver();
    void Parse(unsigned char inData);
	int m_valData[16];
	void Velocity(float inX, float inY, float inAngular);
	void SetSixMotorsSpeed(int *inSpeed);
	int arMotorPos[6];
	int arMotorCurrent[6];
	int nParseCount;

protected:
	unsigned char m_ParseBuf[128];
	int m_nRecvIndex;			
	unsigned char m_lastRecv;	
	bool m_bFrameStart;	
	int m_nFrameLength;	

	int m_nRecvFrameCnt;
	int m_nRecvByteCnt;

	unsigned char m_SendBuf[128];
	unsigned int m_nSendlength;

	float fLinearAccLimit;
	float fAngularAccLimit;

	int m_nMotorToSend[6];

	void m_ParseFrame(unsigned char *inBuf, int inLen);
	float m_CalQuaternionVal(unsigned char *inBuf);
	void m_GenerateSigCmd(int inIndex, unsigned char inID, unsigned char inLen, unsigned char inMode, unsigned char inMethod, unsigned char *data);
	unsigned char m_CalSum(int length);
};