#include "csf.h"
#include "vec3.h"
#include "cloth.h"
#include "rasterization.h"
#include "cdist.h"

//=======================================================================================
CSF::CSF( const int index )
{
    params.sloop_smooth     = true;
    params.time_step        = 0.65;
    params.class_thr        = 0.5;
    params.cloth_resolution = 1;
    params.rigidness        = 3;
    params.iterations      = 500;

    this->index = index;
}
//=======================================================================================
CSF::CSF()
{
    params.sloop_smooth     = true;
    params.time_step        = 0.65;
    params.class_thr        = 0.5;
	params.cloth_resolution = 1;
    params.rigidness        = 3;
    params.iterations      = 500;

    this->index             = 0;
}
//=======================================================================================
CSF::~CSF()
{

}
//=======================================================================================


//=======================================================================================
void CSF::setPointCloud( const std::vector<csf::Point>& points )
{
    _point_cloud.resize( points.size() );

    for ( int i = 0; i < points.size(); i++ )
    {
        csf::Point las;

        las.x = points.at(i).x;
        las.y = - points.at(i).z;
        las.z = points.at(i).y;

        _point_cloud[i] = las;
    }
}
//=======================================================================================


//=======================================================================================
void CSF::setPointCloud( csf::PointCloud& pc )
{
    _point_cloud.resize( pc.size() );

    for ( auto i = 0; i < pc.size(); i++ )
    {
        csf::Point las;

        las.x = pc.at(i).x;
        las.y = - pc.at(i).z;
        las.z = pc.at(i).y;

        _point_cloud[i] = las;
    }
}
//=======================================================================================


//=======================================================================================
void CSF::split( std::vector<int>& ground, std::vector<int>& nonground )
{
    csf::Point pmin;
    csf::Point pmax;



    _point_cloud.set_bounding_box( pmin, pmax );

    auto cloth_y_height = 0.05;
    auto clothbuffer_d = 2;

    Vec3 origin_pos(
        pmin.x - clothbuffer_d * params.cloth_resolution,
        pmax.y + cloth_y_height,
        pmin.z - clothbuffer_d * params.cloth_resolution
    );

    auto width_num = int( std::floor( ( pmax.x - pmin.x ) / params.cloth_resolution ) )
            + 2 * clothbuffer_d;

    auto height_num = int( std::floor( ( pmax.z - pmin.z ) / params.cloth_resolution ) )
            + 2 * clothbuffer_d;

    Cloth cloth(
        origin_pos,
        width_num,
        height_num,
        params.cloth_resolution,
        params.cloth_resolution,
        0.3,
        9999,
        params.rigidness,
        params.time_step
    );

    Rasterization::raster_terrain( cloth, _point_cloud, cloth.getHeightvals() );

    auto time_step_2 = std::pow( params.time_step, 2);
    auto gravity = 0.2;

    cloth.add_force( Vec3( 0, - gravity, 0 ) * time_step_2 );

    for ( auto i = 0; i < params.iterations; i++ )
    {
        auto maxDiff = cloth.time_step();
        cloth.terr_collision();

        if ( ( maxDiff != 0 ) && ( maxDiff < 0.005 ) )
            break;
    }

    if ( params.sloop_smooth )
        cloth.movable_filter();

    c2cdist c2c( params.class_thr );
    c2c.calc_cloud_dist( cloth, _point_cloud, ground, nonground );
}
//=======================================================================================
