/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/
 
#include "wpr1_local_planner/wl_helper.h"
#include <string.h>
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>

#define MAP_WIDTH 100
#define MAP_HEIGHT 100

static unsigned char map_obstacle[MAP_WIDTH*MAP_HEIGHT];
static unsigned char map_target[MAP_WIDTH*MAP_HEIGHT];
static unsigned char map_path[MAP_WIDTH*MAP_HEIGHT];
static unsigned char map_back[MAP_WIDTH*MAP_HEIGHT];
static int loc_path_x[MAP_WIDTH*MAP_HEIGHT];
static int loc_path_y[MAP_WIDTH*MAP_HEIGHT];
static int nLocPathLenght;
static unsigned char TempSinglePoint[21 * 21];
static unsigned char TempSinglePath[255 * 255];
static double x_sin[1081];
static double y_cos[1081];
static int pnt_x[1081];
static int pnt_y[1081];
static float move_x = 0;
static float move_y = 0;

void InitHelper()
{
	//���ݻ���
	memset(map_obstacle, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_target, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_path, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_back, 0, MAP_WIDTH*MAP_HEIGHT);
	//�����״��ģ��
	for (int y = 0; y < 21; y++)
	{
		for (int x = 0; x < 21; x++)
		{
			double fx = x;
			double fy = y;
			int val = sqrt(double(fx - 10)*(fx - 10) + double(fy - 10)*(fy - 10));
			/*if (val > 10)
			{
				val = 10;
			}
			TempSinglePoint[y * 21 + x] = (10 - val) * 20;*/

			if (val < 9)
			{
				TempSinglePoint[y * 21 + x] = 0xff;
			}
			else
			{
				TempSinglePoint[y * 21 + x] = 0;
			}
		}
	}

	//·����ģ��
	memset(TempSinglePath, 0, 255 * 255);
	for (int y = 0; y < 255; y++)
	{
		for (int x = 0; x < 255; x++)
		{
			double fx = x;
			double fy = y;
			int val = sqrt(double(fx - 127)*(fx - 127) + double(fy - 127)*(fy - 127));
			if (val > 127)
			{
				val = 127;
			}
			TempSinglePath[y * 255 + x] = (127 - val) * 1;
		}
	}

	double kStep = (M_PI * 1.5) / 1080;
	for(int i=0;i<1081;i++)
	{
		x_sin[i] = -20 *sin(M_PI*0.75 - kStep*i);
		y_cos[i] = 20 *cos(M_PI*0.75 - kStep*i);
	}
	for (int i = 0; i < 1081; i++)
	{
		pnt_x[i] = 20 * x_sin[i];
		pnt_y[i] = 20 * y_cos[i];
	}
}

void SetObst(int inX, int inY)
{
	if(inX < 0 || inX >= MAP_WIDTH || inY < 0 || inY>=MAP_HEIGHT)
		return;
	//map_obstacle[inY*MAP_WIDTH + inX] = 0xff;

	int nDepthR = 10;
	for (int y = inY - nDepthR; y <= inY + nDepthR; y++)
	{
		for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
		{
			if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
			{
				unsigned char* pMapPoint = map_obstacle + y*MAP_WIDTH + x;
				int tx = x - inX + nDepthR;
				int ty = y - inY + nDepthR;

				unsigned char* pTmpMark = TempSinglePoint + ty * 21 + tx;
				if (*pMapPoint < *pTmpMark)
				{
					*pMapPoint = *pTmpMark;
				}
			}
		}
	}
}

void SetRanges(float* inData)
{
	ClearObst();
	int i =0;
	for(i=0;i<1081;i++)
	{
    	pnt_x[i] = MAP_WIDTH/2 + inData[i] * x_sin[i];
		pnt_y[i] = MAP_HEIGHT/2 + inData[i] * y_cos[i] + 4;
		//if(i == 1080/2)
		//printf("[%d] %.3f (%d , %d)\n",i,inData[i],pnt_x[i],pnt_y[i]);
		SetObst(pnt_x[i],pnt_y[i]);
	}
}

void ClearObst()
{
	memset(map_obstacle, 0, MAP_WIDTH*MAP_HEIGHT);
}

void SetTarget(int inX, int inY)
{
	if(inX >=0 && inX< MAP_WIDTH && inY >= 0 && inY<MAP_HEIGHT)
	{
		map_target[inY*MAP_WIDTH + inX] = 0xff;

		int nDepthR = 126;
		for (int y = inY - nDepthR; y <= inY + nDepthR; y++)
		{
			for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
			{
				if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
				{
					unsigned char* pMapPoint = map_target + y*MAP_WIDTH + x;
					int tx = x - inX + nDepthR;
					int ty = y - inY + nDepthR;

					unsigned char* pTmpMark = TempSinglePath + ty * 255 + tx;
					if (*pMapPoint < *pTmpMark)
					{
						*pMapPoint = *pTmpMark;
					}
				}
			}
		}
	}
}

void ClearTarget()
{
	memset(map_target, 0, MAP_WIDTH*MAP_HEIGHT);
}

bool ChkTarget(int inX, int inY)
{
	bool res = true;
	if(inX >=0 && inX< MAP_WIDTH && inY >= 0 && inY < MAP_HEIGHT)
	{
		if(map_obstacle[inY*MAP_WIDTH+inX] == 0xff)
		{	
			res = false;
		}
	}
	return res;
}

void SetBack(int inX, int inY)
{
	if(inX < 0 || inX >= MAP_WIDTH || inY < 0 || inY>=MAP_HEIGHT)
	return;

	map_back[inY*MAP_WIDTH + inX] = 0xff;

	int nDepthR = 126;
	for (int y = inY - nDepthR; y <= inY + nDepthR; y++)
	{
		for (int x = inX - nDepthR; x <= inX + nDepthR; x++)
		{
			if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
			{
				unsigned char* pMapPoint = map_back + y*MAP_WIDTH + x;
				int tx = x - inX + nDepthR;
				int ty = y - inY + nDepthR;

				unsigned char* pTmpMark = TempSinglePath + ty * 255 + tx;
				if (*pMapPoint < *pTmpMark)
				{
					*pMapPoint = *pTmpMark;
				}
			}
		}
	}
}

int calPointCost(int inX, int inY)
{
	if(inX < 0 || inX >= MAP_WIDTH || inY < 0 || inY>=MAP_HEIGHT)
		return 999;

	int retVal;
	if (map_obstacle[inY*MAP_WIDTH + inX] == 0xff || map_path[inY*MAP_WIDTH + inX] > 0)
	{
		retVal = 999;
	}
	else
	{
		retVal = (int)map_obstacle[inY*MAP_WIDTH + inX] - map_target[inY*MAP_WIDTH + inX];
	}
	return retVal;
}

static int cur_x = 0;
static int cur_y = 0;
bool PathGrow()
{
	bool res = false;
	int tx = cur_x;
	int ty = cur_y;
	int minCost = 256;
	int nx, ny,tCost;
		
	//下
	nx = cur_x + 0;
	ny = cur_y - 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//左下
	nx = cur_x - 1;
	ny = cur_y - 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//右下
	nx = cur_x + 1;
	ny = cur_y - 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//左
	nx = cur_x - 1;
	ny = cur_y - 0;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//右
	nx = cur_x + 1;
	ny = cur_y + 0;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//左上
	nx = cur_x - 1;
	ny = cur_y + 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//右上
	nx = cur_x + 1;
	ny = cur_y + 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//上
	nx = cur_x - 0;
	ny = cur_y + 1;
	tCost = calPointCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}

	cur_x = tx;
	cur_y = ty;
	map_path[cur_y*MAP_WIDTH + cur_x] = 50;
	return res;
}

int calBackCost(int inX, int inY)
{
	int retVal;
	if (map_path[inY*MAP_WIDTH + inX] == 0 || map_back[inY*MAP_WIDTH + inX] == 0)
	{
		retVal = 999;
	}
	else
	{
		retVal = (int)map_back[inY*MAP_WIDTH + inX]*-1;
	}
	return retVal;
}

bool BackGrow()
{
	bool res = false;
	int tx = cur_x;
	int ty = cur_y;
	int minCost = 256;
	int nx, ny;
	//���Ͻ�
	nx = cur_x - 1;
	ny = cur_y + 1;
	int tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//����
	nx = cur_x - 0;
	ny = cur_y + 1;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//����
	nx = cur_x + 1;
	ny = cur_y + 1;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//��
	nx = cur_x + 1;
	ny = cur_y + 0;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//����
	nx = cur_x + 1;
	ny = cur_y - 1;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//��
	nx = cur_x + 0;
	ny = cur_y - 1;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//����
	nx = cur_x - 1;
	ny = cur_y - 1;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}
	//��
	nx = cur_x - 1;
	ny = cur_y - 0;
	tCost = calBackCost(nx, ny);
	if (tCost < minCost)
	{
		tx = nx;
		ty = ny;
		minCost = tCost;
		res = true;
	}

	cur_x = tx;
	cur_y = ty;
	map_path[cur_y*MAP_WIDTH + cur_x] = 255;
	map_back[cur_y*MAP_WIDTH + cur_x] = 0;
	return res;
}

void StartPos()
{
	cur_x = MAP_WIDTH / 2;
	cur_y = MAP_HEIGHT / 2;
	if(map_obstacle[cur_y*MAP_WIDTH + cur_x] == 0xff)
	{
		for(int tr=1;tr<MAP_HEIGHT/2;tr++)
		{
			for(int ty = MAP_HEIGHT/2-tr; ty < MAP_HEIGHT/2+tr; ty++)
			{
				for(int tx = MAP_WIDTH/2-tr; tx < MAP_WIDTH/2+tr; tx++)
				{
					if(tx >=0 && tx< MAP_WIDTH && ty >= 0 && ty < MAP_HEIGHT)
						if(map_obstacle[ty*MAP_WIDTH + tx] != 0xff)
						{
							cur_x = tx;
							cur_y = ty;
							return;
						}
				}
			}
		}
	}
}

bool OutLine()
{
	bool res = false;
	memset(map_path, 0, MAP_WIDTH*MAP_HEIGHT);
	memset(map_back, 0, MAP_WIDTH*MAP_HEIGHT);
	nLocPathLenght = 0;
	StartPos();
	SetBack(cur_x, cur_y);

	// ������
	map_path[cur_y*MAP_WIDTH + cur_x] = 0xff;
	while (PathGrow() == true)
	{
		if (map_target[cur_y*MAP_WIDTH + cur_x] == 0xff)
		{
			res = true;
			break;
		}
	}

	//�������
	map_back[cur_y*MAP_WIDTH + cur_x] = 0;
	while (BackGrow() == true)
	{
		loc_path_x[nLocPathLenght] = cur_x;
		loc_path_y[nLocPathLenght] = cur_y;
		nLocPathLenght++;
		if (map_back[cur_y*MAP_WIDTH + cur_x] == 0xff)
		{
			break;
		}
	}

	//PrintPath();

	// 寻找移动方向
	if(nLocPathLenght > 0)
	{
		int path_index = nLocPathLenght-1;
		while(path_index > 0)
		{
			int nDist = sqrt((loc_path_x[path_index] - MAP_WIDTH/2)*(loc_path_x[path_index] - MAP_WIDTH/2) + (loc_path_y[path_index] - MAP_HEIGHT/2)*(loc_path_y[path_index] - MAP_HEIGHT/2));
			if(nDist > 3)
			{
				break;
			}
			else
			{
				path_index --;
			}
			
		}
		move_x = (loc_path_y[path_index] - MAP_HEIGHT/2) * 0.05;
		move_y = (loc_path_x[path_index] - MAP_WIDTH/2) * 0.05;
		//printf("helper move path=%d (%d,%d)",path_index,loc_path_y[path_index],loc_path_x[path_index]) ;
	}
	else
	{
		move_x = 0;
		move_y = 0;
		res = false;
	}

	return res;
}

void PrintPath()
{
	for (int i = 0; i < nLocPathLenght; i++)
	{
		printf("loc_path [%d] - (%d , %d)\n", i, loc_path_x[i], loc_path_x[i]);
	}
}

float CalAngle(float inX, float inY)
{
	float tan = inY / inX;
	float angle = atan(tan);
	if (inX < 0)
	{
		angle = M_PI + angle;
	}
	if (angle > M_PI)
	{
		angle -= 2 * M_PI;
	}
	return angle;
}

int GetHelperNum()
{
	return nLocPathLenght;
}

float GetFixX()
{
	return move_x;
}

float GetFixY()
{
	return move_y;
}
