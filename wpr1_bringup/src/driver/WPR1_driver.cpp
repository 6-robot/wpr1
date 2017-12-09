#include "WPR1_driver.h"
#include <math.h>

CWPR1_driver::CWPR1_driver()
{
    memset(m_ParseBuf,0,128);
	m_nRecvIndex = 0;
	m_lastRecv = 0;
	m_bFrameStart = false;
	m_nFrameLength = 2;
	m_nRecvFrameCnt = 0;
	m_nRecvByteCnt = 0;

	fLinearAccLimit = 0.2;
	fAngularAccLimit = 0.1;

	for (int i=0;i<6;i++)
	{
		m_nMotorToSend[i] = 0;
		arMotorPos[i] = 0;
		arMotorCurrent[i] = 0;
	}
	nParseCount = 1000;
}
    
CWPR1_driver::~CWPR1_driver()
{

}

float CWPR1_driver::m_CalQuaternionVal(unsigned char *inBuf)
{
	int nTmp = m_Piece2int(inBuf);
	float retVal = (float)nTmp ;
	return retVal;
}

void CWPR1_driver::Parse(unsigned char inData)
{
	m_nRecvByteCnt ++;
	m_ParseBuf[m_nRecvIndex] = inData;
	
	if (m_lastRecv == 0x55 && inData == 0xAA && m_bFrameStart == 0)
	{
        //printf("[SerialCom]m_bFrameStart = true;\n");
		m_bFrameStart = true;
		m_ParseBuf[0] = m_lastRecv;
		m_ParseBuf[1] = inData;
		m_nRecvIndex = 2;
		m_lastRecv=0x00;
		return;
	}
	
	if (m_bFrameStart)
	{
		if (m_nRecvIndex == 3)
		{
			m_nFrameLength = inData + 8;
		}
		//put received data into buffer
		m_ParseBuf[m_nRecvIndex]=inData;
		m_nRecvIndex++;
		
		//receive one frame, invoke ParseFrame to parse
		if (m_nRecvIndex==m_nFrameLength)
		{ 
            //printf("[SerialCom]m_ParseFrame\n");
			m_ParseFrame(m_ParseBuf, m_nFrameLength);
			m_bFrameStart = false;
		}
		
		//receive buffer overflow
		if (m_nRecvIndex>=128) 
		{
			//m_ResetRcvBuf();
			m_bFrameStart = false;
		}
	}
	else
		m_lastRecv=inData;
}

short m_ShortFromChar(unsigned char *inBuf)
{
	short wtemp;
	wtemp = 0;
	wtemp |= *(inBuf);

	wtemp <<= 8;
	wtemp |= *(inBuf + 1);

	return wtemp;
}

static float fAngle = 180/3.1415;
void CWPR1_driver::m_ParseFrame(unsigned char *inBuf, int inLen)
{
	int i;
	switch (inBuf[2])		//����ID
	{
	case 0x38:
		switch (inBuf[4])	//ģ��
		{
		case 0x07:		//AD
			switch (inBuf[5])
			{
			case 0x60:
				for (i = 0; i<8; i++)
				{
					m_valData[i] = m_USFromChar(&(inBuf[7 + i * 2]));
				}
				break;

			case 0x61:
				for (i = 0; i<8; i++)
				{
					m_valData[i + 8] = m_USFromChar(&(inBuf[7 + i * 2]));
				}
				break;
			}
			break;
		case 0x08:		//Motor
			for (i=0;i<4;i++)
			{
				arMotorPos[i] = m_Piece2int(&(inBuf[7+i*6]));
				arMotorCurrent[i] = m_ShortFromChar(&(inBuf[7+i*6+4]));
			}
			//printf("M[1]= %.4d  M[2]= %.4d M[3]= %.4d\n", arMotorPos[0],arMotorPos[1],arMotorPos[2]);
			// printf("C[1]= %.4d  C[2]= %.4d C[3]= %.4d \n", arMotorCurrent[0],arMotorCurrent[1],arMotorCurrent[2]);
			// printf("[Recv] ");
			// for (int i = 0; i<inLen; i++)
			// {
			// 	printf("%.2X ", inBuf[i]);
			// }
			// printf("\n");
			break;
		}//end of switch(inBuf[4])
		break;
	}
	nParseCount = 0;
}

static float vkx = (float)sqrt(3.0f)*0.5;
static float fKLinearMotorK = 1276.6;
static float fKAngularMotorK = 285;
void CWPR1_driver::Velocity(float inX, float inY, float inAngular)
{
	//upward backward
	int nVectorX = inX * fKLinearMotorK;

	//shift left right
	int nVectorY = inY * fKLinearMotorK;

	//Turning 
	int nVectorTurn = inAngular * fKAngularMotorK;
	//Speed Value
	m_nMotorToSend[0] = 0;	//left front
	m_nMotorToSend[0] = -vkx*nVectorX + nVectorY*0.5 + nVectorTurn;

	m_nMotorToSend[1] = 0;	//right front
	m_nMotorToSend[1] = vkx*nVectorX + nVectorY*0.5 + nVectorTurn;

	m_nMotorToSend[2] = 0;	//back 
	m_nMotorToSend[2] = -nVectorY + nVectorTurn;

	m_nMotorToSend[3] = 0;	//NC
	m_nMotorToSend[4] = 0;	//NC

	//printf("[CWPR1_driver::Velocity]-> [0]%d [1]%d [2]%d [3]%d \n", m_nMotorToSend[0], m_nMotorToSend[1], m_nMotorToSend[2], m_nMotorToSend[3]);

	SetSixMotorsSpeed(m_nMotorToSend);
}


void CWPR1_driver::SetSixMotorsSpeed(int *inSpeed)
{
	static unsigned char speedbuf[12];
	static short tmpShort = 0;

	tmpShort = inSpeed[0];
	m_Split2Bytes(&speedbuf[0], tmpShort);

	//printf("inSpeed[0] = %d  tmpShort = %d \n",inSpeed[0],tmpShort);

	tmpShort = inSpeed[1];
	m_Split2Bytes(&speedbuf[2], tmpShort);
	//printf("inSpeed[1] = %d  tmpShort = %d \n",inSpeed[1],tmpShort);

	tmpShort = inSpeed[2];
	m_Split2Bytes(&speedbuf[4], tmpShort);
	//printf("inSpeed[2] = %d  tmpShort = %d \n",inSpeed[2],tmpShort);

	tmpShort = inSpeed[3];
	m_Split2Bytes(&speedbuf[6], tmpShort);

	tmpShort = inSpeed[4];
	m_Split2Bytes(&speedbuf[8], tmpShort);

	// if (inSpeed[5] < 0)
	// {
	// 	inSpeed[5] = 0;
	// }
	// if (inSpeed[5] > 40000)
	// {
	// 	inSpeed[5] = 40000;
	// }
	// WORD tmpWord = inSpeed[5];
	// speedbuf[10] = (tmpWord >> 8) & 0x00ff;
	// speedbuf[11] = tmpWord & 0x00ff;

	m_GenerateSigCmd(1, 0x38, 12, 0x08, 0x70, speedbuf);
}

void CWPR1_driver::m_GenerateSigCmd(int inIndex, unsigned char inID, unsigned char inLen, unsigned char inMode, unsigned char inMethod, unsigned char *data)
{
	m_SendBuf[0] = (unsigned char)0x55;
	m_SendBuf[1] = (unsigned char)0xaa;
	m_SendBuf[2] = inID;
	m_SendBuf[3] = inLen;
	m_SendBuf[4] = inMode;
	m_SendBuf[5] = inMethod;
	if (inLen>0 && data != NULL)
	{
		memcpy(&m_SendBuf[6], data, inLen);
	}
	m_SendBuf[inLen + 6] = m_CalSum(inLen + 6);

	m_nSendlength = inLen + 7;

	Send(m_SendBuf, m_nSendlength);

	// for (int i = 0; i<m_nSendlength; i++)
	// {
	// 	printf("[m_GenerateSigCmd] ");
	// 	printf("%.2X ", m_SendBuf[i]);
	// 	printf("\n");
	// }
}

unsigned char CWPR1_driver::m_CalSum(int length)
{
	int temp = 0;
	for (int i = 0; i<length; i++)
		temp += m_SendBuf[i];

	unsigned char ret;
	ret = (unsigned char)(temp & 0x000000ff);
	return ret;
}
