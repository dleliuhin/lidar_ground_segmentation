#ifndef RASTERIZATION_H
#define RASTERIZATION_H

#include "point_cloud.h"
#include "cloth.h"

//=======================================================================================
class Rasterization
{
public:

    Rasterization() {}
    ~Rasterization() {}

    //-----------------------------------------------------------------------------------

    double static find_height_neighbor( Particle* p, Cloth& cloth );
    double static find_height_scanline( Particle* p, Cloth& cloth );

    void static raster_terrain( Cloth& cloth,
                                csf::PointCloud& pc,
                                std::vector<double>& heightVal );

    //-----------------------------------------------------------------------------------

private:

    static double _square_dist( const double x1, const double y1,
                                const double x2, const double y2);

};
//=======================================================================================

#endif // RASTERIZATION_H
