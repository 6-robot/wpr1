#include "Mani_driver.h"
#include <math.h>

CMani_driver::CMani_driver()
{
    memset(m_SendBuf, 0, 128);
	memset(m_ParseBuf, 0, 128);
	m_nRecvIndex = 0;			//接收索引
	m_lastRecv = 0;				//上一个字符
	m_bFrameStart = false;		//帧解析开始
	m_nFrameLength = 14;			//帧长度

	for (int i = 0; i < 7; i++)
	{
		nRecvJointPos[i] = 0;
		nRecvJointCurrent[i] = 0;
	}
	nRecvJointPos[3] = -9000;
	nRecvJointPos[4] = -4000;

	nValidFrameCount = 0;
}
    
CMani_driver::~CMani_driver()
{

}

void CMani_driver::Parse(unsigned char inData)
{
	m_ParseBuf[m_nRecvIndex] = inData;

	if (m_lastRecv == 0x55 && inData == 0xAA && m_bFrameStart == 0)
	{
		m_bFrameStart = 1;
		m_ParseBuf[0] = m_lastRecv;
		m_ParseBuf[1] = inData;
		m_nRecvIndex = 2;
		m_lastRecv = 0x00;
		return;
	}

	if (m_bFrameStart)
	{
		if (m_nRecvIndex == 4)
		{
			m_nFrameLength = inData + 6;
		}
		
		//put received data into buffer
		m_ParseBuf[m_nRecvIndex] = inData;
		m_nRecvIndex++;

		//receive one frame, invoke ParseFrame to parse
		if (m_nRecvIndex == m_nFrameLength)
		{
			//m_DispRecvCmd();
			m_ParseFrame();
			m_bFrameStart = false;
		}

		//receive buffer overflow
		if (m_nRecvIndex >= 128)
		{
			//m_ResetRcvBuf();
			m_bFrameStart = 0;
		}
	}
	else
		m_lastRecv = inData;
}


void CMani_driver::m_Split2Bytes(unsigned char *inTarg, short inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned short temp;
	memcpy(&temp, &inSrc, sizeof(short));
	inTarg[1] = (unsigned char)temp & 0x00ff;

	temp >>= 8;

	inTarg[0] = (unsigned char)temp & 0x00ff;
}


void CMani_driver::m_Split4Bytes(unsigned char *inTarg, int inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned int temp;
	memcpy(&temp, &inSrc, sizeof(int));
	inTarg[3] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[2] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[1] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[0] = (unsigned char)temp & 0x00ff;
}


short CMani_driver::m_WordFromChar(unsigned char *inBuf)
{
	static short wtemp;
	wtemp = 0;
	wtemp = *(inBuf);

	wtemp <<= 8;
	wtemp |= *(inBuf + 1);

	return wtemp;
}

int CMani_driver::m_IntFromChar(unsigned char *inBuf)
{
	static int itemp;
	itemp = 0;
	itemp = *(inBuf);

	itemp <<= 8;
	itemp |= *(inBuf + 1);

	itemp <<= 8;
	itemp |= *(inBuf + 2);

	itemp <<= 8;
	itemp |= *(inBuf + 3);

	return itemp;
}

void CMani_driver::m_CalSum()
{
	int nLen = m_SendBuf[4] + 6;

	m_SendBuf[nLen-1] = 0x00;
	for (int i = 0; i < nLen - 1; i++)
	{
		m_SendBuf[nLen - 1] += m_SendBuf[i];
	}
}

void CMani_driver::m_ParseFrame()
{
	if (m_ParseBuf[2] == 0x02)
	{
		nValidFrameCount++;
		for (int i = 0; i < 7; i++)
		{
			nRecvJointCurrent[i] = m_IntFromChar(&m_ParseBuf[5 + i * 8]);
			nRecvJointPos[i] = m_IntFromChar(&m_ParseBuf[5 + i * 8 + 4]);
		}
		//////////////////////
		//手爪特殊处理
		nRecvJointPos[6] = 50000 - nRecvJointPos[6];
		if (nRecvJointPos[6] < 0)
		{
			nRecvJointPos[6] = 0;
		}
		if (nRecvJointPos[6] > 50000)
		{
			nRecvJointPos[6] = 50000;
		}
		//////////////////////
		// printf("[电机位置] ");
		// for (int i = 0; i < 7; i++)
		// {
		// 	printf("关节%d = %.6d ", i + 1, nRecvJointPos[i]);
		// }
		// printf("\n");

		// if (pDisJointCurrent != NULL)
		// {
		// 	str = L"[关节电机电流] ";
		// 	for (int i = 0; i < 7; i++)
		// 	{
		// 		strTmp.Format(L"关节%d = %.10d     ", i + 1, nRecvJointCurrent[i]);
		// 		str += strTmp;
		// 	}
		// 	pDisJointCurrent->SetWindowText(str);
		// }
	}

	m_DisRecv();
}


void CMani_driver::m_DisRecv()
{
	// if (pRecvList != NULL)
	// {
	// 	CString strData, str;
	// 	strData = "";
	// 	int i;
	// 	for (i = 0; i < m_nFrameLength; i++)
	// 	{
	// 		str.Format(L"%.2X ", m_ParseBuf[i]);
	// 		strData += str;
	// 	}
	// 	pRecvList->AddString(strData);
	// 	pRecvList->SetCurSel(pRecvList->GetCount() - 1);
	// }
}


void CMani_driver::SetSpeed(int inSpeed)
{
	m_SendBuf[0] = 0x55;
	m_SendBuf[1] = 0xAA;
	m_SendBuf[2] = 0x01; //id
	m_SendBuf[3] = 0x00; //cntl
	m_SendBuf[4] = 0x04; //len

	m_Split4Bytes(&m_SendBuf[5], inSpeed);

	m_CalSum();

	Send(m_SendBuf, 10);
}

//发送机械臂控制指令
//0是升降,0~350
//3\4\5是手臂,-180~180
//6是手爪,25000
static int arPosition[7];
void CMani_driver::SetJoints(double* inPosition, int* inSpeed)
{
	
	m_SendBuf[0] = 0x55;
	m_SendBuf[1] = 0xAA;
	m_SendBuf[2] = 0x02; //id
	m_SendBuf[3] = 0x00; //cntl
	m_SendBuf[4] = 7*8; //len

	//升降
	arPosition[0] = inPosition[0];
	m_Split4Bytes(&m_SendBuf[5], inSpeed[0]);
	m_Split4Bytes(&m_SendBuf[9], arPosition[0]);

	//角度
	for (int i = 1; i < 6; i++)
	{
		float tmpAngle = inPosition[i];
		while (tmpAngle > 179.99)
		{
			tmpAngle -= 360;
		}
		while (tmpAngle < -180)
		{
			tmpAngle += 360;
		}
		arPosition[i] = tmpAngle * 100;

		m_Split4Bytes(&m_SendBuf[5+i*8], inSpeed[i]);
		m_Split4Bytes(&m_SendBuf[9+i*8], arPosition[i]);
	}

	//手爪
	arPosition[6] = inPosition[6];
	//////////////
	//临时修正
	arPosition[6] = 50000 - arPosition[6];
	if (arPosition[6] < 0)
	{
		arPosition[6] = 0;
	}
	if (arPosition[6] > 47000)
	{
		arPosition[6] = 47000;
	}
	///////////////
	m_Split4Bytes(&m_SendBuf[5 + 6*8], inSpeed[6]);
	m_Split4Bytes(&m_SendBuf[9 + 6 * 8], arPosition[6]);

	m_CalSum();

	Send(m_SendBuf, m_SendBuf[4] + 6);
}


void CMani_driver::Set5Dof(double* inPos, int* inSpeed)
{

	m_SendBuf[0] = 0x55;
	m_SendBuf[1] = 0xAA;
	m_SendBuf[2] = 0x02; //id
	m_SendBuf[3] = 0x00; //cntl
	m_SendBuf[4] = 5 * 8; //len

	//角度
	for (int i = 0; i < 5; i++)
	{
		float tmpAngle = inPos[i];
		while (tmpAngle > 179.99)
		{
			tmpAngle -= 360;
		}
		while (tmpAngle < -180)
		{
			tmpAngle += 360;
		}
		arPosition[i] = tmpAngle * 100;

		m_Split4Bytes(&m_SendBuf[5 + i * 8], inSpeed[i]);
		m_Split4Bytes(&m_SendBuf[9 + i * 8], arPosition[i]);
	}

	m_CalSum();

	Send(m_SendBuf, m_SendBuf[4] + 6);
}