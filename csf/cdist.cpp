#include "cdist.h"
#include <cmath>

//=======================================================================================
CDist::CDist( double threshold )
    : _threshold ( threshold )
{

}
//=======================================================================================


//=======================================================================================
void CDist::calc_cloud_dist( Cloth& cloth,
                               const csf::PointCloud& pc,
                               QVector<int>& ground,
                               QVector<int>& nonground )
{
    ground.resize(0);
    nonground.resize(0);

    for ( auto i = 0; i < pc.count(); i++ )
    {
        auto pc_x = pc.at(i).x;
        auto pc_z = pc.at(i).z;

        auto delta_x = pc_x - cloth.origin_pos().f()[0];
        auto delta_z = pc_z - cloth.origin_pos().f()[2];

        auto col0 = int( delta_x / cloth.step_x() );
        auto row0 = int( delta_z / cloth.step_y() );
        auto col1 = col0 + 1;
        auto row1 = row0;
        auto col2 = col0 + 1;
        auto row2 = row0 + 1;
        auto col3 = col0;
        auto row3 = row0 + 1;

        auto subdeltaX = ( delta_x - col0 * cloth.step_x() ) / cloth.step_x();
        auto subdeltaZ = ( delta_z - row0 * cloth.step_y() ) / cloth.step_y();

        auto fxy = cloth.get_particle( col0, row0 )->pos().f()[1] *
                ( 1 - subdeltaX ) * ( 1 - subdeltaZ ) +
                cloth.get_particle( col3, row3 )->pos().f()[1] *
                ( 1 - subdeltaX ) * subdeltaZ +
                cloth.get_particle( col2, row2 )->pos().f()[1] *
                subdeltaX * subdeltaZ +
                cloth.get_particle( col1, row1 )->pos().f()[1] *
                subdeltaX * ( 1 - subdeltaZ );

        auto height_var = fxy - pc[i].y;

        if ( std::fabs( height_var ) < _threshold )
            ground.push_back(i);

        else
            nonground.push_back(i);
    }
}
//=======================================================================================
