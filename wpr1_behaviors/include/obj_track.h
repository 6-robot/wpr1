#ifndef WPR_OBJ_TEACK_H_
#define WPR_OBJ_TEACK_H_

#include <math.h>

#define PNT_MAX 512*424 

class CObjTrack
{
public:
    CObjTrack();
    virtual ~CObjTrack();
    void GetObjPosition();
    float Cal2DDist(float inX_1, float inY_1, float inX_2, float inY_2);

public:
    float* pntx;
    float* pnty;
    float* pntz;
    int nPointNum;

    float fPlaneHeight;

    float obj_x;
    float obj_y;
    float obj_z;
};


#endif 