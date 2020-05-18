#ifndef CDIST_H
#define CDIST_H

#include "cloth.h"
#include "point_cloud.h"

//=======================================================================================
class CDist
{
public:

    CDist( double threshold );

    //-----------------------------------------------------------------------------------

    void calc_cloud_dist( Cloth& cloth,
                          const csf::PointCloud& pc,
                          QVector<int>& ground,
                          QVector<int>& nonground );

    //-----------------------------------------------------------------------------------

private:

    double _threshold;
};
//=======================================================================================

#endif // ifndef _C2CDIST_H_
