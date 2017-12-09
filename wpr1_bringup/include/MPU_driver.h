#include "SerialCom.h"
#pragma once

class CMPU_driver : public CSerialCom
{
public:
    CMPU_driver();
    ~CMPU_driver();
    void Parse(unsigned char inData);

	unsigned char m_ParseBuf[128];
	int m_nRecvIndex;			//��������
	unsigned char m_lastRecv;	//��һ���ַ�
	bool m_bFrameStart;			//֡������ʼ
	int m_nFrameLength;			//֡����

	int m_nRecvFrameCnt;
	int m_nRecvByteCnt;

	float fQuatW;
	float fQuatX;
	float fQuatY;
	float fQuatZ;
	
	float fGyroX;
	float fGyroY;
	float fGyroZ;
	
	float fAccX;
	float fAccY;
	float fAccZ;

	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float m_fYaw;
	float m_fPitch;
	float m_fRoll;

protected:
	void m_ParseFrame(unsigned char *inBuf, int inLen);
	float m_CalQuaternionVal(unsigned char *inBuf);
};