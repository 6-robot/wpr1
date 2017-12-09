#include "MPU_driver.h"
#include <math.h>

CMPU_driver::CMPU_driver()
{
    memset(m_ParseBuf,0,128);
	m_nRecvIndex = 0;
	m_lastRecv = 0;
	m_bFrameStart = false;
	m_nFrameLength = 43;

	fQuatW = 0.0f;
	fQuatX = 0.0f;
	fQuatY = 0.0f;
	fQuatZ = 0.0f;
	fGyroX = 0.0f;
	fGyroY = 0.0f;
	fGyroZ = 0.0f;
	fAccX = 0.0f;
	fAccY = 0.0f;
	fAccZ = 0.0f;
	
	m_nRecvFrameCnt = 0;
	m_nRecvByteCnt = 0;

	ypr[0] = 0;
	ypr[1] = 0;
	ypr[2] = 0;
}
    
CMPU_driver::~CMPU_driver()
{

}

float CMPU_driver::m_CalQuaternionVal(unsigned char *inBuf)
{
	int nTmp = m_Piece2int(inBuf);
	float retVal = (float)nTmp ;
	return retVal;
}

#define int16_t int
#define uint32_t unsigned int
#define uint8_t unsigned char
struct Quaternion_
{ 
    float   w,x,y,z;
};
struct VectorFloat
{ 
    float x,y,z;
};
struct VectorInt16
{ 
    int   x,y,z;
};
uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
	//    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
	if (data[0]>32768)
		data[0]=-65536+data[0];
	
    data[1] = ((packet[4] << 8) + packet[5]);
	if (data[1]>32768)
		data[1]=-65536+data[1];
	
    data[2] = ((packet[8] << 8) + packet[9]);
	if (data[2]>32768)
		data[2]=-65536+data[2];
	
    data[3] = ((packet[12] << 8) + packet[13]);
	if (data[3]>32768)
		data[3]=-65536+data[3];
    return 0;
}
uint8_t dmpGetQuaternion_(Quaternion_ *q, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) 
    {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
uint8_t dmpGetGravity(VectorFloat *v, Quaternion_ *q) 
{
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    //if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}
uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    //if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[28] << 24) | ((uint32_t)packet[29] << 16) | ((uint32_t)packet[30] << 8) | packet[31]);
    data[1] = (((uint32_t)packet[32] << 24) | ((uint32_t)packet[33] << 16) | ((uint32_t)packet[34] << 8) | packet[35]);
    data[2] = (((uint32_t)packet[36] << 24) | ((uint32_t)packet[37] << 16) | ((uint32_t)packet[38] << 8) | packet[39]);
    return 0;
}

uint8_t dmpGetAccel_(VectorInt16 *v, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
	//    if (packet == 0) packet = dmpPacketBuffer;
	int16_t AAdata[3];
	dmpGetAccel(AAdata,packet);
    v -> x = AAdata[0];
    v -> y = AAdata[1];
    v -> z = AAdata[2];
    return 0;
}
uint8_t dmpGetYawPitchRoll(float *data, Quaternion_ *q, VectorFloat *gravity) 
{
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}
// uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) 
{
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x - gravity -> x*4096*2;
    v -> y = vRaw -> y - gravity -> y*4096*2;
    v -> z = vRaw -> z - gravity -> z*4096*2;
    return 0;
}
static Quaternion_ q_;           // [w, x, y, z]         quaternion container
static VectorFloat gravity;    // [x, y, z]            gravity vector
static float ypr_p[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
static int GYRO_available=0;

void CMPU_driver::Parse(unsigned char inData)
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

static float fAngle = 180/3.1415;
void CMPU_driver::m_ParseFrame(unsigned char *inBuf, int inLen)
{
	m_nRecvFrameCnt ++;
	unsigned char ckSum = 0;
	for (int i=0;i<42;i++ )
	{
		ckSum += inBuf[i];
	}
	if (ckSum != inBuf[42])
	{
		return;
	}
	unsigned char* pBuf = inBuf + 2;
	fQuatW = m_CalQuaternionVal(&pBuf[0]);
	fQuatX = m_CalQuaternionVal(&pBuf[4]);
	fQuatY = m_CalQuaternionVal(&pBuf[8]);
	fQuatZ = m_CalQuaternionVal(&pBuf[12]);
	//printf("[CMPU_driver] qw= %d   qx= %d   qy= %d  qz= %d\n",nQuatW,nQuatX,nQuatY,nQuatZ);
	
	// fGyroX = 2 * (fQuatX*fQuatZ - fQuatW*fQuatY);
    // fGyroY = 2 * (fQuatW*fQuatX + fQuatY*fQuatZ);
    // fGyroZ = fQuatW*fQuatW - fQuatX*fQuatX - fQuatY*fQuatY + fQuatZ*fQuatZ;
	fGyroX = (float)m_Piece2int(&pBuf[16]);
	fGyroY = (float)m_Piece2int(&pBuf[20]);
	fGyroZ = (float)m_Piece2int(&pBuf[24]);
	//printf("[CMPU_driver] gx= %d   gy= %d  gz= %d\n",nGyroX,nGyroY,nGyroZ);
	
	
	fAccX = (float)m_Piece2int(&pBuf[28]);
	fAccY = (float)m_Piece2int(&pBuf[32]);
	fAccZ = (float)m_Piece2int(&pBuf[36]);
	//printf("[CMPU_driver] ax= %5d   ay= %5d  az= %5d\n",nAccX,nAccY,nAccZ);

	// dmpGetQuaternion_(&q_, pBuf);
	// dmpGetGravity(&gravity, &q_);
	// dmpGetYawPitchRoll(ypr, &q_, &gravity);
	
	// yaw: (about Z axis)
    m_fYaw = atan2(2*fQuatX*fQuatY - 2*fQuatW*fQuatZ, 2*fQuatW*fQuatW + 2*fQuatX*fQuatX - 1);
    // pitch: (nose up/down, about Y axis)
    m_fPitch = atan(fGyroY / sqrt(fGyroX*fGyroX + fGyroZ*fGyroZ));
    // roll: (tilt left/right, about X axis)
   	m_fRoll = atan(fGyroX / sqrt(fGyroY*fGyroY + fGyroZ*fGyroZ));
    // m_fYaw = m_fYaw*fAngle;
	// m_fPitch = m_fPitch*fAngle;
	// m_fRoll = m_fRoll*fAngle;

	/////////////
	
		// printf("[CMPU_driver] fQuatX = %f | q_.x = %f\n",fQuatX , q_.x);
		// printf("[CMPU_driver] fQuatY = %f | q_.y = %f\n",fQuatY , q_.y);
		// printf("[CMPU_driver] fQuatZ = %f | q_.z = %f\n",fQuatZ , q_.z);
		// printf("[CMPU_driver] fQuatW = %f | q_.w = %f\n",fQuatW , q_.w);
	
		// printf("[CMPU_driver] fGyroX = %f | gravity.x = %f\n",fGyroX , gravity.x);
		// printf("[CMPU_driver] fGyroy = %f | gravity.y = %f\n",fGyroY , gravity.y);
		// printf("[CMPU_driver] fGyroz = %f | gravity.z = %f\n",fGyroZ , gravity.z);

		// printf("[CMPU_driver] m_fYaw = %f | ypr[0] = %f\n",m_fYaw , ypr[0]);
		// printf("[CMPU_driver] m_fPith = %f | ypr[1] = %f\n",m_fPitch , ypr[1]);
		// printf("[CMPU_driver] m_fRolll = %f | ypr[2] = %f\n",m_fRoll , ypr[2]);
	////////////////
	int nYaw = m_fYaw*fAngle*2;
	int nPitch = m_fPitch*fAngle;
	int nRoll = m_fRoll*fAngle;
	// int nYaw = ypr[0]*fAngle;
	// int nPitch = ypr[1]*fAngle;
	// int nRoll = ypr[2]*fAngle;
	//printf("[CMPU_driver] Yaw= %3d   Pitch= %3d  Roll= %3d\n",nYaw,nPitch,nRoll);
	//dmpGetAccel_(&aa, m_pMPU6050Data);
	//dmpGetLinearAccel(&aaReal, &aa, &gravity);	//aaReal_G[0]=aaReal.x;	aaReal_G[1]=aaReal.y;	aaReal_G[2]=aaReal.z;
	
}