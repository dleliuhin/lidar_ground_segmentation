#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include "point_cloud.h"
#include "cloth.h"

//=======================================================================================
class Rasterization
{
public:

    double static find_height_neighbor( Particle* p );
    double static find_height_scanline( Particle* p, Cloth& cloth );

    void static raster_terrain( Cloth& cloth,
                                csf::PointCloud& pc,
                                QVector<double>& heightVal );

    //-----------------------------------------------------------------------------------

private:

    static double _square_dist( double x1, double y1, double x2, double y2);

};
//=======================================================================================

#endif // RASTERIZATION_H
