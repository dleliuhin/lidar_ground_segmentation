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

        for ( auto d = 0; d < 3; ++d )
        {
            if ( P.u[d] < pmin.u[d] )
                pmin.u[d] = P.u[d];

            else if ( P.u[d] > pmax.u[d] )
                pmax.u[d] = P.u[d];
        }
    }
}
//=======================================================================================
