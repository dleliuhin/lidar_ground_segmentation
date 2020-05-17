#include "rasterization.h"
#include <queue>

//=======================================================================================
double Rasterization::find_height_scanline( Particle *p, Cloth& cloth )
{
    auto xpos = p->pos_x();
    auto ypos = p->pos_y();

    for ( auto i = xpos + 1; i < cloth.particles_width(); i++ )
    {
        auto crresHeight = cloth.get_particle( i, ypos )->nearest_point_height();

        if ( crresHeight > min_inf )
            return crresHeight;
    }

    for ( auto i = xpos - 1; i >= 0; i-- )
    {
        auto crresHeight = cloth.get_particle( i, ypos )->nearest_point_height();

        if ( crresHeight > min_inf )
            return crresHeight;
    }

    for ( auto j = ypos - 1; j >= 0; j-- )
    {
        auto crresHeight = cloth.get_particle( xpos, j )->nearest_point_height();

        if ( crresHeight > min_inf )
            return crresHeight;
    }

    for ( auto j = ypos + 1; j < cloth.particles_height(); j++ )
    {
        auto crresHeight = cloth.get_particle( xpos, j )->nearest_point_height();

        if ( crresHeight > min_inf )
            return crresHeight;
    }

    return find_height_neighbor(p);
}
//=======================================================================================
double Rasterization::find_height_neighbor( Particle* p )
{
    std::queue<Particle*>  nqueue;
    std::vector<Particle*> pbacklist;

    auto neiborsize = p->neighbors_list().count();

    for ( auto i = 0; i < neiborsize; i++ )
    {
        p->is_visited( true );
        nqueue.push( p->neighbors_list()[i] );
    }

    while ( !nqueue.empty() )
    {
        auto pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back( pneighbor );

        if ( pneighbor->nearest_point_height() > min_inf )
        {
            for ( auto& p: pbacklist )
                p->is_visited( false );

            while ( !nqueue.empty() )
            {
                auto pp = nqueue.front();
                pp->is_visited( false );
                nqueue.pop();
            }

            return pneighbor->nearest_point_height();
        }

        auto nsize = pneighbor->neighbors_list().size();

        for ( auto i = 0; i < nsize; i++ )
        {
            auto ptmp = pneighbor->neighbors_list()[i];

            if ( !ptmp->is_visited() )
            {
                ptmp->is_visited( true );
                nqueue.push( ptmp );
            }
        }
    }

    return min_inf;
}
//=======================================================================================
void Rasterization::raster_terrain( Cloth& cloth,
                                    csf::PointCloud& pc,
                                    QVector<double>& heightVal )
{
    for ( auto i = 0; i < pc.size(); i++ )
    {
        auto pc_x = pc[i].x;
        auto pc_z = pc[i].z;

        auto deltaX = pc_x - cloth.origin_pos().f()[0];
        auto deltaZ = pc_z - cloth.origin_pos().f()[2];
        auto col    = int(deltaX / cloth.step_x() + 0.5);
        auto row    = int(deltaZ / cloth.step_y() + 0.5);

        if ( ( col >= 0 ) && ( row >= 0 ) )
        {
            auto pt = cloth.get_particle( col, row );

            pt->corresponding_point_list().push_back(i);

            auto pc2particleDist = _square_dist( pc_x,
                                                 pc_z,
                                                 pt->get_pos().f()[0],
                    pt->get_pos().f()[2] );

            if ( pc2particleDist < pt->tmp_dist() )
            {
                pt->tmp_dist( pc2particleDist );
                pt->nearest_point_height( pc[i].y );
                pt->nearest_point_id(i);
            }
        }
    }

    heightVal.resize( cloth.get_size() );

    for ( auto i = 0; i < cloth.get_size(); i++ )
    {
        auto pcur = cloth.getParticle1d(i);
        auto nearestHeight = pcur->nearest_point_height();

        if ( nearestHeight > min_inf )
            heightVal[i] = nearestHeight;

        else
            heightVal[i] = find_height_scanline( pcur, cloth );
    }
}
//=======================================================================================


//=======================================================================================
double Rasterization::_square_dist( double x1, double y1, double x2, double y2 )
{
    return ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 );
}
//=======================================================================================
