#ifndef CSF_H
#define CSF_H

#include <vector>
#include <string>

#include "point_cloud.h"

//=======================================================================================
struct Params
{
    bool sloop_smooth { true };
    double time_step { 0.65 };
    double class_thr { 0.5 };
    double cloth_resolution {1};
    int rigidness {3};
    int iterations { 500 };
};
//=======================================================================================

//=======================================================================================
class CSF
{
public:

    CSF( int _index );
    CSF() = default;

    void params( const Params& other );

    void setPointCloud( const QVector<csf::Point>& points );

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


    void split( QVector<int>& ground_idx, QVector<int>& no_ground_idx );

    //-----------------------------------------------------------------------------------

private:

    Params _params;
    int _index {0};

    csf::PointCloud _point_cloud;

};

#endif // CSF_H
