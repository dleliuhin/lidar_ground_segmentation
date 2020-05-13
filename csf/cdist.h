#ifndef CDIST_H
#define CDIST_H

#include "cloth.h"
#include "point_cloud.h"

//=======================================================================================
class c2cdist
{
public:

    c2cdist( const double threshold )
        : _threshold ( threshold )
    {

    }

    ~c2cdist() {}

    //-----------------------------------------------------------------------------------

    void calc_cloud_dist( Cloth& cloth,
                          const csf::PointCloud& pc,
                          std::vector<int>& ground,
                          std::vector<int>& nonground );

    //-----------------------------------------------------------------------------------

private:

    double _threshold;
};
//=======================================================================================

#endif // ifndef _C2CDIST_H_
