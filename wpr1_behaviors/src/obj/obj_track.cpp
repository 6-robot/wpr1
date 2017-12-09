#include "obj_track.h"
#include <stdlib.h>
#include <stdio.h>

CObjTrack::CObjTrack()
{
    pntx = new float[PNT_MAX];
    pnty = new float[PNT_MAX];
    pntz = new float[PNT_MAX];
    nPointNum = 0;
    fPlaneHeight = 0.8f;

    obj_x = 0;
    obj_y = 0;
    obj_z = 0;
}

CObjTrack::~CObjTrack()
{
    delete []pntx;
    delete []pnty;
    delete []pntz;
}
    
float CObjTrack::Cal2DDist(float inX_1, float inY_1, float inX_2, float inY_2)
{
    float res_dist = sqrt( (inX_1 - inX_2)*(inX_1 - inX_2) + (inY_1 - inY_2)*(inY_1 - inY_2));
    return res_dist;
}

void CObjTrack::GetObjPosition()
{
    float t_range = 0.1;
    float t_height = 0.1;
    double x_sum = 0;
    double y_sum = 0;
    double x_min = 0;
    int nObjPntCount = 0;
    for (int i = 0; i < nPointNum; i++)
    {
        if( 
            pntz[i] > (fPlaneHeight+0.02) && 
            pntz[i] < (fPlaneHeight+0.02 + t_height) &&
            //Cal2DDist(pntx[i] , obj_x ,pnty[i] , obj_y) < t_range
            pntx[i] > obj_x - t_range && pntx[i] < obj_x + t_range &&
            pnty[i] > obj_y - t_range && pnty[i] < obj_y + t_range
            )
        {
            //符合条件的点
            if(x_sum < 0.001 && y_sum < 0.001 && pntx[i] > 0)
            {
                //第一个点
                x_min = pntx[i];
            }
            if( pntx[i] < x_min && pntx[i] > 0)
            {
                x_min = pntx[i];
            }

            x_sum += pntx[i];
            y_sum += pnty[i];
            nObjPntCount ++;

        //printf("[CObjTrack] point(%d) = ( %.2f , %.2f , %.2f) o(%.2f ,%.2f)) \n",i,pntx[i],pnty[i],pntz[i],obj_x,obj_y);
        }
    }
    //printf("[CObjTrack] nPointNum = %d , nObjPntCount= %d \n",nPointNum,nObjPntCount);
    if(nObjPntCount > 0)
    {
        obj_x = x_min;
        obj_y = y_sum/nObjPntCount;
    }
}