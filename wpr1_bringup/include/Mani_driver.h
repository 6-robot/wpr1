#include "SerialCom.h"
#pragma once

class CMani_driver : public CSerialCom
{
public:
    CMani_driver();
	~CMani_driver();
    void m_Split2Bytes(unsigned char *inTarg, short inSrc);
	void m_Split4Bytes(unsigned char *inTarg, int inSrc);
	short m_WordFromChar(unsigned char *inBuf);
	int m_IntFromChar(unsigned char *inBuf);

	void Parse(unsigned char inData);
	void SetSpeed(int inSpeed);

	unsigned char m_SendBuf[128];
	unsigned char m_ParseBuf[128];
	int m_nRecvIndex;			//接收索引
	unsigned char m_lastRecv;	//上一个字符
	bool m_bFrameStart;			//帧解析开始
	int m_nFrameLength;			//帧长度

	void m_CalSum();

	int nValidFrameCount;
	int nRecvJointPos[7];
	int nRecvJointCurrent[7];

	void m_ParseFrame();

	void m_DisRecv();
	void SetJoints(double* inPosition, int* inSpeed);
	void Set5Dof(double* inPos, int* inSpeed);
};