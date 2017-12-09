
#include "dock_detect.h"
#include <math.h> 
#include <string.h>

#define	 DOCK_MAP_TITLE	"���׮"
#define  DOCK_MAP_WIDTH 300
#define  DOCK_MAP_HEIGHT 300

#ifdef DETECTDOCK_CV_SHOW
#include <opencv2/opencv.hpp>
static IplImage* imageDockDetect;
#endif

static double m_pi = 3.1415926;
static double m_kDist = 0.2;	//ÿ�����ȵ�λ5mm
static double m_kAngle = m_pi / 180;
static int arDistVal[1081];

static int arNewLidarPoint_x[1081];
static int arNewLidarPoint_y[1081];

static double arTmpNoRotAngle[1081];

//���������
#define POINT_R 10
static int nDepthR = POINT_R;
static unsigned char TempSinglePoint[(POINT_R * 2 + 1) * (POINT_R * 2 + 1)];
static bool arEnable[1081];
static unsigned char* arMap;

//�����ģ��
static unsigned char DockTemplate[360][DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT];
static unsigned char DockShape[DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT];

//����ƥ��ֵ���õı���
static int nIntegralXOffset = 0;
static int nIntegralYOffset = 0;
static int nIntegralAngleOffset = 0;

//����ƥ��
static int arLidarPointMatch[360][1081];
static int nDockMatchSum = 0;


static int nFaceDist = 70;

static double AngleFix(double inAngle, double inMin, double inMax)
{
	if (inMax - inMin > 360)
	{
		//PrintInfo(L"[CDataHolder::AngleFix]������Χ����360");
		return inAngle;
	}
	int retAngle = inAngle;
	while (retAngle < inMin)
	{
		retAngle += 360;
	}
	while (retAngle > inMax)
	{
		retAngle -= 360;
	}
	return retAngle;
}

///////////////////////////////////////////////

CDetectDock::CDetectDock()
{
	arMap = new unsigned char[DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT];
	memset(arMap, 0, DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT);

	nFace_x = 10;
	nFace_y = 0;

	//��ʼ����ת��
	double kStepAngle = 270.0f / 1080;
	for (int i = 0; i < 1081; i++)
	{
		arTmpNoRotAngle[i] = 135 - kStepAngle*i;
		arEnable[i] = true;
		for (int j = 0; j < 360;j++)
		{
			arLidarPointMatch[j][i] = 0;
		}
	}

	for (int y = 0; y < (POINT_R * 2 + 1); y++)
	{
		for (int x = 0; x < (POINT_R * 2 + 1); x++)
		{
			double fx = x;
			double fy = y;
			int val = sqrt(double(fx - POINT_R)*(fx - POINT_R) + double(fy - POINT_R)*(fy - POINT_R));
			if (val > POINT_R)
			{
				val = POINT_R;
			}
			TempSinglePoint[y * (POINT_R * 2 + 1) + x] = (POINT_R - val) * (250 / POINT_R);
		}
	}

	for (int i = 0; i < 360;i++)
	{
		for (int j = 0; j < DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT;j++)
		{
			DockTemplate[i][j] = 0;
		}
	}
	memset(DockShape, 0, DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT);

	InitDockSize(410, 173);

	/////////////
#ifdef DETECTDOCK_CV_SHOW
	imageDockDetect = cvCreateImage(cvSize(DOCK_MAP_WIDTH, DOCK_MAP_HEIGHT), IPL_DEPTH_8U, 3);
	cvZero(imageDockDetect);
	cvNamedWindow(DOCK_MAP_TITLE, 0/*CV_WINDOW_AUTOSIZE*/);
	cvMoveWindow(DOCK_MAP_TITLE, 18, 200);
	cvResizeWindow(DOCK_MAP_TITLE, DOCK_MAP_WIDTH, DOCK_MAP_HEIGHT);
#endif
	/////////////

	nDock_X = 0;
	nDock_Y = 180;
	fDock_Angle = 0;

	bFindDock = true;
}


CDetectDock::~CDetectDock()
{
	delete[]arMap;
	/////////////
#ifdef DETECTDOCK_CV_SHOW
	cvDestroyAllWindows();
	if (imageDockDetect != NULL)
	{
		cvReleaseImage(&imageDockDetect);
	}
#endif
	/////////////
}


static bool chkXYValid(int inX, int inY)
{
	bool res = true;
	if (inX < 0 || inX >= DOCK_MAP_WIDTH || inY < 0 || inY >= DOCK_MAP_HEIGHT)
	{
		return false;
	}
	else
	{
		return true;
	}
}


static void /*CDetectDock::*/DrawPointInMap(int inX, int inY)
{
	//��ģ���н������
	for (int y = inY - nDepthR; y <= inY + nDepthR; y++)
	{
		for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
		{
			if (x >= 0 && x < DOCK_MAP_WIDTH && y >= 0 && y < DOCK_MAP_HEIGHT)
			{
				unsigned char* pMapPoint = arMap + y*DOCK_MAP_WIDTH + x;
				int tx = x - inX + nDepthR;
				int ty = y - inY + nDepthR;

				unsigned char* pTmpMark = TempSinglePoint + ty * (POINT_R * 2 + 1) + tx;
				if (*pMapPoint < *pTmpMark)
				{
					*pMapPoint = *pTmpMark;
				}
			}
		}
	}
}


static void /*CDetectDock::*/DrawDock(int inX, int inY, int inAngle)
{
	int nDock_Map_x = inX + DOCK_MAP_WIDTH / 2;
#ifdef DETECTDOCK_CV_SHOW
	int tmpAngle = inAngle;
	while (tmpAngle < 0)
	{
		tmpAngle += 360;
	}
	while (tmpAngle >= 360)
	{
		tmpAngle -= 360;
	}
	int nFlipY = DOCK_MAP_HEIGHT - inY;

	unsigned char* pCvBuf = (unsigned char*)imageDockDetect->imageDataOrigin;
	for (int y = 0; y < DOCK_MAP_HEIGHT; y++)
	{
		for (int x = 0; x < DOCK_MAP_WIDTH; x++)
		{
			if (DockTemplate[tmpAngle][y*DOCK_MAP_WIDTH + x] > 0)
			{
				int nCvX = nDock_Map_x + x - DOCK_MAP_WIDTH / 2;
				int nCvy = nFlipY + y - DOCK_MAP_WIDTH / 2;
				if (chkXYValid(nCvX, nCvy) == true)
				{
					pCvBuf[(nCvy*DOCK_MAP_WIDTH + nCvX) * 3 + 0] = 0;
					pCvBuf[(nCvy*DOCK_MAP_WIDTH + nCvX) * 3 + 1] = 0;
					pCvBuf[(nCvy*DOCK_MAP_WIDTH + nCvX) * 3 + 2] = 0xff;
				}
			}
		}
	}
#endif
}


static void /*CDetectDock::*/m_CvShow(int inDockX, int inDockY, int inDockAngle, int inFacex,int inFacey)
{
#ifdef DETECTDOCK_CV_SHOW
	//[1]��ģ��ͼ���Ƶ�ͼ����
	unsigned char* pCvBuf = (unsigned char*)imageDockDetect->imageDataOrigin;
	for (size_t i = 0; i < DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT; i++)
	{
		pCvBuf[3 * i + 0] = arMap[i];
		pCvBuf[3 * i + 1] = arMap[i];
		pCvBuf[3 * i + 2] = arMap[i];
	}
	//[2]���������״����ʾ����
	DrawDock(inDockX, inDockY, inDockAngle);

	//[3]������������������
	CvPoint pntStart = cvPoint(inDockX + DOCK_MAP_WIDTH/2, DOCK_MAP_HEIGHT - inDockY);
	CvPoint pntEnd = cvPoint(inFacex + DOCK_MAP_WIDTH / 2, DOCK_MAP_HEIGHT - inFacey);
	CvScalar Clr = cvScalar(255, 255, 0);
	cvLine(imageDockDetect, pntStart, pntEnd, Clr, 1, CV_AA);
	
	cvShowImage(DOCK_MAP_TITLE, imageDockDetect);
#endif
}

//���ݳ����λ�ü����泯���ĵ㣨�����ǳ���������λ�ã�
static void GenFacePoint(int inDockX, int inDockY, int inDockAngle, int* inpFacex, int* inpFacey)
{
	double fAngle = -inDockAngle*m_pi / 180;
	double fDx = nFaceDist * sin(fAngle);
	double fDy = nFaceDist * cos(fAngle);
	*inpFacex = inDockX + fDx;
	*inpFacey = inDockY - fDy;
}

//�������
void CDetectDock::InData(int* inDistVal)
{
	double tmpAngle;
	for (int i = 0; i < 1081; i++)
	{
		arDistVal[i] = inDistVal[i];

		tmpAngle = (0 + arTmpNoRotAngle[i])*m_kAngle;
		arNewLidarPoint_x[i] = 0 + arDistVal[i] * m_kDist * sin(tmpAngle);
		arNewLidarPoint_y[i] = 0 + arDistVal[i] * m_kDist * cos(tmpAngle);
	}

	//[1]���״�����ƥ��ͼ��
	memset(arMap, 0, DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT);
	int nMidWidth = DOCK_MAP_WIDTH / 2;
	for (int i = 0; i < 1081; i++)
	{
		if (arEnable[i] == true)
		{
			DrawPointInMap(arNewLidarPoint_x[i] + nMidWidth, DOCK_MAP_HEIGHT - arNewLidarPoint_y[i]);
		}
	}

	//[2]�����µ�ƥ������
	GenNewDockPos();
	
	//[3]����Ҫȥ����Ŀ��
	GenFacePoint(nDock_X, nDock_Y, fDock_Angle, &nFace_x, &nFace_y);

	//[n]��ʾ
	m_CvShow(nDock_X, nDock_Y, fDock_Angle,nFace_x,nFace_y);

#ifdef MFC_LIST
	CString strInfo;
	strInfo.Format(L"(%d,%d) %.2f M=%d - (%d,%d)",
		nDock_X,
		nDock_Y,
		fDock_Angle,
		nDockMatchSum,
		nFace_x,
		nFace_y);
	pInfoList->AddString(strInfo);
	pInfoList->SetCurSel(pInfoList->GetCount() - 1);
#endif
}


static double m_CalDirectAngle(int inX, int inY)
{
	double res = 0;

	int dx = 0 - inX;
	double dy = 0 - inY;
	if (dx == 0)
	{
		if (dy > 0)
		{
			res = 180 + 90;
		}
		else
		{
			res = 0 + 90;
		}

	}
	else
	{
		double fTan = dy / dx;
		res = atan(fTan) * 180 / 3.1415926;

		if (dx > 0)
		{
			res = res - 180;
		}

	}
	//res -= 90;

	if (res < 0)
	{
		res += 360;
	}

	if (res > 360)
	{
		res -= 360;
	}

	return res;
}
void CDetectDock::InitDockSize(int inLength, int inWing)
{
	//[1]���ԭ����ģ��
	for (int i = 0; i < 360; i++)
	{
		for (int j = 0; j < DOCK_MAP_WIDTH* DOCK_MAP_HEIGHT; j++)
		{
			DockTemplate[i][j] = 0;
		}
	}
	//[2]�ȳ�ʼ��0����ת��ʱ��
	double fScale = 0.20;
	int nScaleLenght = inLength*fScale;
	int nScaleWing = inWing*fScale;
	int nLenXMin, nLenXMax, nLenY;
	nLenXMin = (DOCK_MAP_WIDTH - nScaleLenght) / 2;
	nLenXMax = (DOCK_MAP_WIDTH + nScaleLenght) / 2;
	nLenY = DOCK_MAP_HEIGHT / 2;
	//�ױ�
	for (int x = nLenXMin; x < nLenXMax;x++)
	{
		DockTemplate[0][nLenY*DOCK_MAP_WIDTH + x] = 0xff;
	}
	//����
	double fSin30 = sin(m_pi / 6);
	double fCos30 = cos(m_pi / 6);
	for (int i = 0; i < nScaleWing; i++)
	{
		int dx = fSin30*i;
		int dy = fCos30*i;
		if (chkXYValid(nLenXMin - dx, nLenY + dy) == true)
		{
			DockTemplate[0][(nLenY + dy)*DOCK_MAP_WIDTH + (nLenXMin - dx)] = 0xff;
		}
		if (chkXYValid(nLenXMin + dx, nLenY + dy) == true)
		{
			DockTemplate[0][(nLenY + dy)*DOCK_MAP_WIDTH + (nLenXMax + dx)] = 0xff;
		}
	}

	//[3]��ת����360��ģ��
	for (int y = 0; y < DOCK_MAP_HEIGHT; y++)
	{
		for (int x = 0; x < DOCK_MAP_WIDTH; x++)
		{
			if (DockTemplate[0][y*DOCK_MAP_WIDTH + x] > 0)
			{
				int dx = x - DOCK_MAP_WIDTH / 2;
				int dy = y - DOCK_MAP_HEIGHT / 2;

				double dist = sqrt((double)(dx*dx + dy*dy)); 
				double fCurAngle = m_CalDirectAngle(dx,dy);
				
				for (int i = 1; i < 360; i++)
				{
					double fAngle = (fCurAngle + i)*m_pi / 180;
					double fSin = sin(fAngle);
					double fCos = cos(fAngle);
					int nNewx = DOCK_MAP_WIDTH / 2 + dist*fCos;
					int nNewy = DOCK_MAP_HEIGHT / 2 + dist*fSin;

					if (chkXYValid(nNewx, nNewy) == true)
					{
						DockTemplate[i][nNewy*DOCK_MAP_WIDTH + nNewx] = DockTemplate[0][y*DOCK_MAP_WIDTH + x];
					}
				}
			}
		}
	}
}


int CDetectDock::m_CalMatchSum(int inX, int inY, int inAngle)
{
	int tmpAngle = inAngle;
	while (tmpAngle < 0)
	{
		tmpAngle += 360;
	}
	while (tmpAngle >= 360)
	{
		tmpAngle -= 360;
	}
	int nFlipY = DOCK_MAP_HEIGHT - inY;

	int nSum = 0;

	for (int y = 0; y < DOCK_MAP_HEIGHT; y++)
	{
		for (int x = 0; x < DOCK_MAP_WIDTH; x++)
		{
			if (DockTemplate[tmpAngle][y*DOCK_MAP_WIDTH + x] > 0)
			{
				int nCvX = inX + x - DOCK_MAP_WIDTH / 2;
				int nCvy = nFlipY + y - DOCK_MAP_HEIGHT / 2;
				if (chkXYValid(nCvX, nCvy) == true)
				{
					nSum += arMap[nCvy*DOCK_MAP_WIDTH + nCvX];
				}
			}
		}
	}

	return nSum;
}


void CDetectDock::GenNewDockPos()
{
	if (bFindDock == true)
	{
		int nMidWidth = DOCK_MAP_WIDTH / 2;
		int nBestDockX = 0;
		int nBestDockY = 0;
		int nBestDockAngle = 0;
		int nBestMatch = 0;
		//��һ֡,�������м���㣬Ѱ����ƥ���λ��
		for (int i = 0; i < 1081;i++)
		{
			int x = arNewLidarPoint_x[i] + nMidWidth;
			int y = /*DOCK_MAP_HEIGHT - */arNewLidarPoint_y[i];
			if (chkXYValid(x, y) == true)
			{
				for (int j = 0; j<15; j++)
				{
					arLidarPointMatch[j][i] = m_CalMatchSum(x, y, j);
					if (arLidarPointMatch[j][i] > nBestMatch)
					{
						nBestMatch = arLidarPointMatch[j][i];
						nBestDockX = x;
						nBestDockY = y;
						nBestDockAngle = j;
					}
				}

				for (int j = 345; j < 360; j++)
				{
					arLidarPointMatch[j][i] = m_CalMatchSum(x, y, j);
					if (arLidarPointMatch[j][i] > nBestMatch)
					{
						nBestMatch = arLidarPointMatch[j][i];
						nBestDockX = x;
						nBestDockY = y;
						nBestDockAngle = j;
					}
				}
			} 
			else
			{
				for (int j = 0; j < 360; j++)
				{
					arLidarPointMatch[j][i] = 0;
				}
			}
		}
		nDock_X = nBestDockX - DOCK_MAP_WIDTH/2;
		nDock_Y = nBestDockY;
		fDock_Angle = nBestDockAngle;
		bFindDock = false;
		return;
	}

	nIntegralXOffset = 0;
	nIntegralYOffset = 0;
	nIntegralAngleOffset = 0;
	int nBestXOffset = 0;
	int nBestYOffset = 0;
	int nBestAngleOffset = 0;
	int nBestMatchSum = 0;
	int nMapDock_x = nDock_X + DOCK_MAP_WIDTH / 2;
	do 
	{
		//��������
		//��������ƶ���ƥ��ȣ���Ϊ�Ƚϵı�׼
		nBestMatchSum = m_CalMatchSum(nMapDock_x + nIntegralXOffset, nDock_Y + nIntegralYOffset, fDock_Angle + nIntegralAngleOffset);
		nBestXOffset = 0;
		nBestYOffset = 0;
		nBestAngleOffset = 0;
		for (int x = -1; x <= 1;x++)
		{
			for (int y = -1; y <= 1;y++)
			{
				for (int angle = -1; angle <= 1;angle ++)
				{
					//���㳢�Եĵ�����ƥ���
					int nTmpMatchSum = m_CalMatchSum(nMapDock_x + nIntegralXOffset + x, nDock_Y + nIntegralYOffset + y, fDock_Angle + nIntegralAngleOffset + angle);
					if (nTmpMatchSum > nBestMatchSum)
					{
						//����ƥ�����ߵĵ���ƥ�����
						nBestMatchSum = nTmpMatchSum;
						nBestXOffset = x;
						nBestYOffset = y;
						nBestAngleOffset = angle;
					}
				}
			}
		}
		//�ۼӵ������
		nIntegralXOffset += nBestXOffset;
		nIntegralYOffset += nBestYOffset;
		nIntegralAngleOffset += nBestAngleOffset;
	} while (nBestXOffset != 0 || nBestYOffset != 0 || nBestAngleOffset != 0 );

	//���������µ�����
	nIntegralXOffset += nMapDock_x;
	nDock_X = nIntegralXOffset - DOCK_MAP_WIDTH/2;
	nIntegralXOffset = 0;

	nIntegralYOffset += nDock_Y;
	nDock_Y = nIntegralYOffset;
	nIntegralYOffset = 0;

	nIntegralAngleOffset += fDock_Angle;
	fDock_Angle = nIntegralAngleOffset;
	nIntegralAngleOffset = 0;

	//��¼���ƥ��ֵ����ͨ��������ж�ƥ��Ч�������ж�ƥ�䵽���Ƿ��ǳ���룩
	nDockMatchSum = nBestMatchSum;
}


void CDetectDock::Reset()
{
	bFindDock = true;
}


void CDetectDock::SetDist(int inDist)
{
	nFaceDist = inDist;
}


int CDetectDock::GetDistToPoint(int inTargetX ,int inTargetY)
{
	int nDistToPoint = 0;

	nDistToPoint = sqrt((double)(inTargetX*inTargetX + inTargetY*inTargetY));

	return nDistToPoint;
}
