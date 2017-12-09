/*!
 @abstract   串口通讯
 @author     张万杰
 */

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义**/
#include <string.h>
#include <pthread.h>

#pragma once

class CSerialCom
{
public:
    CSerialCom();
    virtual ~CSerialCom();
    void Open(const char* inDev, int inBaudRate);
    static void* threadSerialCom_func(void* inParam);
	int m_Piece2int(unsigned char *inTarg);
    int m_Piece2short(unsigned char *inTarg);
    unsigned short m_USFromChar(unsigned char *inBuf);
    void m_Split2Bytes(unsigned char *inTarg, short inSrc);
	virtual void Parse(unsigned char inData){};
	int ReadNewData();
	int Send(unsigned char* inBuf,int inLen);

    char* buffer;
    int fdCom;

protected:
    pthread_t hComThread;
	unsigned int GetBaudRate(int inBaudRate);

};
