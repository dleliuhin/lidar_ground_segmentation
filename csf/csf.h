#ifndef CSF_H
#define CSF_H

#include <vector>
#include <string>

#include "point_cloud.h"

//=======================================================================================
struct Params
{
    bool sloop_smooth;
    double time_step;
    double class_thr;
    double cloth_resolution;
    int rigidness;
    int iterations;
};
//=======================================================================================

//=======================================================================================
class CSF
{
public:

    Params params;
    int index;

    //-----------------------------------------------------------------------------------

    CSF( const int index );
    CSF();
    ~CSF();

    void setPointCloud( const std::vector<csf::Point>& points );

    inline csf::PointCloud & getPointCloud()
    {
        return _point_cloud;
    }

    inline const csf::PointCloud & getPointCloud() const
    {
        return _point_cloud;
    }

    // get size of pointcloud
    std::size_t size()
    {
        return _point_cloud.size();
    }

    void setPointCloud( csf::PointCloud& pc );


    void split( std::vector<int>& ground_idx, std::vector<int>& no_ground_idx );

    //-----------------------------------------------------------------------------------

private:

    csf::PointCloud _point_cloud;

};

#endif // CSF_H
