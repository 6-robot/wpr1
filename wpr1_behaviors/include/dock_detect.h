#pragma once

//#define DETECTDOCK_CV_SHOW
//#define MFC_LIST

class CDetectDock
{
public:
	CDetectDock();
	~CDetectDock();

	int nFace_x;
	int nFace_y;

	int nDock_X;
	int nDock_Y;
	double fDock_Angle;

	bool bFindDock;

#ifdef MFC_LIST
	CListBox * pInfoList;
#endif

	void SetDist(int inDist);
	void InData(int* inDistVal);
	void InitDockSize(int inLength, int inWing);
	void Reset();
	int GetDistToPoint(int inTargetX, int inTargetY);
private:
	int m_CalMatchSum(int inX, int inY, int inAngle);
	void GenNewDockPos();
};

