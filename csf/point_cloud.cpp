#include "point_cloud.h"

//=======================================================================================
void csf::PointCloud::set_bounding_box( Point& pmin, Point& pmax )
{
    if ( empty() )
    {
        pmin = pmax = Point();
        return;
    }

    pmin = pmax = at(0);

    for ( auto i = 1; i < size(); i++ )
    {
        const auto P = at(i);

        if ( P.x < pmin.x )
            pmin.x = P.x;

        else if ( P.x > pmax.x )
            pmax.x = P.x;

        if ( P.y < pmin.y )
            pmin.y = P.y;

        else if ( P.y > pmax.y )
            pmax.y = P.y;

        if ( P.z < pmin.z )
            pmin.z = P.z;

        else if ( P.z > pmax.z )
            pmax.z = P.z;
    }
}
//=======================================================================================
