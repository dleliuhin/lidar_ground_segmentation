#include "cloth.h"

static constexpr auto postprocess_max_particles = 50;

#include <queue>

//=======================================================================================
XY::XY( int x, int y )
{
    _x = x;
    _y = y;
}
//=======================================================================================

//=======================================================================================
int XY::x() const
{
    return _x;
}
//=======================================================================================
int XY::y() const
{
    return _y;
}
//=======================================================================================


//=======================================================================================
Cloth::Cloth( Vec3 &origin_pos,
              int pwidth,
              int pheight,
              double step_x,
              double step_y,
              double smooth_thr,
              double height_thr,
              int rigidness,
              double time_step )
    : _origin_pos        ( origin_pos )
    , _step_x            ( step_x     )
    , _step_y            ( step_y     )
    , _particles_width   ( pwidth     )
    , _particles_height  ( pheight    )
    , _constraint_iters ( rigidness   )
    , _smooth_thr       ( smooth_thr )
    , _height_thr       ( height_thr )
{
    _particles.resize( pwidth * pheight );

    auto time_step_2 = pow( time_step, 2 );

    for ( auto i = 0; i < pwidth; i++ )
        for ( auto j = 0; j < pheight; j++ )
        {
            Vec3 pos( origin_pos.f().at(0) + i * step_x,
                      origin_pos.f().at(1),
                      origin_pos.f().at(2) + j * step_y );

            _particles[ j * pwidth + i ] = Particle( pos, time_step_2 );
            _particles[ j * pwidth + i ].pos_x(i);
            _particles[ j * pwidth + i ].pos_y(j);
        }

    for ( auto x = 0; x < pwidth; x++ )
        for ( auto y = 0; y < pheight; y++ )
        {
            if ( x < pwidth - 1 )
                make_constraint( get_particle( x, y ), get_particle( x + 1, y ) );

            if ( y < pheight - 1 )
                make_constraint( get_particle( x, y ), get_particle( x, y + 1 ) );

            if ( ( x < pwidth - 1 ) && ( y < pheight - 1 ) )
                make_constraint( get_particle( x, y ), get_particle( x + 1, y + 1 ) );

            if ( ( x < pwidth - 1 ) && ( y < pheight - 1 ) )
                make_constraint( get_particle( x + 1, y ), get_particle( x, y + 1 ) );
        }

    for ( auto x = 0; x < pwidth; x++ )
        for ( auto y = 0; y < pheight; y++ )
        {
            if ( x < pwidth - 2 )
                make_constraint( get_particle( x, y ), get_particle( x + 2, y ) );

            if ( y < pheight - 2 )
                make_constraint( get_particle( x, y ), get_particle( x, y + 2 ) );

            if ( ( x < pwidth - 2 ) && ( y < pheight - 2 ) )
                make_constraint( get_particle( x, y ), get_particle( x + 2, y + 2 ) );

            if ( ( x < pwidth - 2 ) && ( y < pheight - 2 ) )
                make_constraint( get_particle( x + 2, y ), get_particle( x, y + 2 ) );
        }
}
//=======================================================================================


//=======================================================================================
const Vec3 & Cloth::origin_pos() const
{
    return _origin_pos;
}
//=======================================================================================
double Cloth::step_x() const
{
    return _step_x;
}
//=======================================================================================
double Cloth::step_y() const
{
    return _step_y;
}
//=======================================================================================
int Cloth::particles_width() const
{
    return _particles_width;
}
//=======================================================================================
int Cloth::particles_height() const
{
    return _particles_height;
}
//=======================================================================================


//=======================================================================================
double Cloth::time_step()
{
    for ( auto& p: _particles )
        p.time_step();

    for ( auto& p: _particles )
        p.satisfy_constraint_self( _constraint_iters );

    auto max_diff = 0.0;

    for ( auto& p: _particles )
        if ( p.is_movable() )
        {
            auto diff = fabs( p.old_pos().f().at(1) - p.pos().f().at(1) );

            if ( diff > max_diff )
                max_diff = diff;
        }

    return max_diff;
}
//=======================================================================================
void Cloth::add_force( const Vec3& direction )
{
    for ( auto& p: _particles )
        p.add_force( direction );
}
//=======================================================================================
void Cloth::terr_collision()
{
    for ( auto i = 0; i < _particles.size(); i++ )
    {
        Vec3 v = _particles[i].get_pos();

        if ( v.f().at(1) < _heightvals[i] )
        {
            _particles[i].offset_pos( { 0, _heightvals[i] - v.f().at(1), 0 } );
            _particles[i].make_unmovable();
        }
    }
}
//=======================================================================================
void Cloth::movable_filter()
{
    for ( int x = 0; x < _particles_width; x++ )
        for ( int y = 0; y < _particles_height; y++ )
        {
            auto ptc = get_particle( x, y );

            if ( ptc->is_movable() && !ptc->is_visited() )
            {
                std::queue<int> que;
                QVector<XY> connected;
                QVector<QVector<int>> neighbors;
                auto sum = 1;
                auto index = y * _particles_width + x;

                connected.push_back( { x, y } );
                _particles[ index ].is_visited( true );

                que.push( index );

                while ( !que.empty() )
                {
                    Particle *ptc_f = &_particles[ que.front() ];
                    que.pop();
                    auto cur_x = ptc_f->pos_x();
                    auto cur_y = ptc_f->pos_y();
                    QVector<int> neighbor;

                    if ( cur_x > 0 )
                    {
                        auto ptc_left = get_particle(cur_x - 1, cur_y);

                        if ( ptc_left->is_movable() )
                        {
                            if ( !ptc_left->is_visited() )
                            {
                                sum++;
                                ptc_left->is_visited( true );
                                connected.push_back( { cur_x - 1, cur_y } );
                                que.push( _particles_width * cur_y + cur_x - 1 );
                                neighbor.push_back( sum - 1 );
                                ptc_left->c_pos( sum - 1 );
                            }

                            else
                                neighbor.push_back( ptc_left->c_pos() );
                        }
                    }

                    if ( cur_x < _particles_width - 1 )
                    {
                        auto ptc_right = get_particle( cur_x + 1, cur_y );

                        if ( ptc_right->is_movable() )
                        {
                            if ( !ptc_right->is_visited() )
                            {
                                sum++;
                                ptc_right->is_visited( true );
                                connected.push_back( { cur_x + 1, cur_y } );
                                que.push( _particles_width * cur_y + cur_x + 1 );
                                neighbor.push_back( sum - 1 );
                                ptc_right->c_pos( sum - 1 );
                            }

                            else
                                neighbor.push_back( ptc_right->c_pos() );
                        }
                    }

                    if ( cur_y > 0 )
                    {
                        auto ptc_bottom = get_particle( cur_x, cur_y - 1 );

                        if ( ptc_bottom->is_movable() )
                        {
                            if ( !ptc_bottom->is_visited() )
                            {
                                sum++;
                                ptc_bottom->is_visited( true );
                                connected.push_back( { cur_x, cur_y - 1 } );
                                que.push( _particles_width * ( cur_y - 1 ) + cur_x );
                                neighbor.push_back( sum - 1 );
                                ptc_bottom->c_pos( sum - 1 );
                            }

                            else
                                neighbor.push_back( ptc_bottom->c_pos() );
                        }
                    }

                    if ( cur_y < _particles_height - 1 )
                    {
                        auto ptc_top = get_particle( cur_x, cur_y + 1 );

                        if ( ptc_top->is_movable() )
                        {
                            if ( !ptc_top->is_visited() )
                            {
                                sum++;
                                ptc_top->is_visited( true );
                                connected.push_back( { cur_x, cur_y + 1 } );
                                que.push( _particles_width * ( cur_y + 1 ) + cur_x );
                                neighbor.push_back( sum - 1 );
                                ptc_top->c_pos( sum - 1 );
                            }

                            else
                                neighbor.push_back( ptc_top->c_pos() );
                        }
                    }
                    neighbors.append( neighbor );
                }

                if ( sum > postprocess_max_particles )
                {
                    auto edge_points = find_unmovable( connected );
                    handle_slop_connected( edge_points, connected, neighbors );
                }
            }
        }
}
//=======================================================================================
QVector<int> Cloth::find_unmovable( const QVector<XY>& connected )
{
    QVector<int> edge_points;

    for ( auto i = 0; i < connected.count(); i++ )
    {
        auto x = connected.at(i).x();
        auto y = connected.at(i).y();
        auto index = y * _particles_width + x;

        auto ptc = get_particle( x, y );

        if ( x > 0 )
        {
            auto ptc_x = get_particle( x - 1, y );

            if ( !ptc_x->is_movable() )
            {
                auto index_ref = y * _particles_width + x - 1;

                auto cond_1 = fabs( _heightvals.at( index ) -
                                    _heightvals.at( index_ref ) ) < _smooth_thr;
                auto cond_2 = ptc->get_pos().f().at(1) - _heightvals.at( index ) < _height_thr;

                if ( cond_1 && cond_2 )
                {
                    auto offset_vec = Vec3( 0,
                                            _heightvals.at( index ) - ptc->get_pos().f().at(1),
                                            0 );
                    _particles[ index ].offset_pos( offset_vec );
                    ptc->make_unmovable();
                    edge_points.push_back(i);

                    continue;
                }
            }
        }

        if ( x < _particles_width - 1 )
        {
            auto ptc_x = get_particle( x + 1, y );

            if ( !ptc_x->is_movable() )
            {
                auto index_ref = y * _particles_width + x + 1;

                auto cond_1 = fabs( _heightvals.at( index ) -
                                    _heightvals.at( index_ref ) ) < _smooth_thr;

                auto cond_2 = ptc->get_pos().f()[1] - _heightvals[index] < _height_thr;

                if ( cond_1 && cond_2 )
                {
                    auto offsetVec = Vec3( 0,
                                           _heightvals[index] - ptc->get_pos().f()[1], 0 );
                    _particles[index].offset_pos( offsetVec );
                    ptc->make_unmovable();
                    edge_points.push_back(i);

                    continue;
                }
            }
        }

        if ( y > 0 )
        {
            auto ptc_y = get_particle( x, y - 1 );

            if ( !ptc_y->is_movable() )
            {
                auto index_ref = ( y - 1 ) * _particles_width + x;

                auto cond_1 = fabs( _heightvals.at( index ) -
                                    _heightvals.at( index_ref ) ) < _smooth_thr;

                auto cond_2 = ( ptc->get_pos().f().at(1) -
                                _heightvals.at( index ) < _height_thr );

                if ( cond_1 && cond_2 )
                {
                    auto offsetVec = Vec3( 0,
                                           _heightvals.at( index ) - ptc->get_pos().f().at(1),
                                           0 );
                    _particles[index].offset_pos( offsetVec );
                    ptc->make_unmovable();
                    edge_points.push_back(i);

                    continue;
                }
            }
        }

        if ( y < _particles_height - 1 )
        {
            auto ptc_y = get_particle( x, y + 1 );

            if ( !ptc_y->is_movable() )
            {
                auto index_ref = ( y + 1 ) * _particles_width + x;

                auto cond_1 = fabs( _heightvals.at( index ) -
                                    _heightvals.at( index_ref ) ) < _smooth_thr;

                auto cond_2 = ( ptc->get_pos().f().at(1) -
                                _heightvals.at( index ) < _height_thr );

                if ( cond_1 && cond_2 )
                {
                    auto offsetVec = Vec3( 0,
                                           _heightvals.at( index ) - ptc->get_pos().f().at(1),
                                           0 );
                    _particles[index].offset_pos( offsetVec );
                    ptc->make_unmovable();
                    edge_points.push_back(i);

                    continue;
                }
            }
        }
    }

    return edge_points;
}
//=======================================================================================
void Cloth::handle_slop_connected( const QVector<int>& edge_points,
                                   const QVector<XY>& connected,
                                   const QVector<QVector<int>>& neighbors )
{
    QVector<bool> visited( connected.count() );
    visited.fill( false );

    std::queue<int> que;

    for ( auto i = 0; i < edge_points.count(); i++ )
    {
        que.push( edge_points[i] );
        visited[edge_points[i]] = true;
    }

    while ( !que.empty() )
    {
        auto index = que.front();
        que.pop();

        auto index_center = connected.at( index ).y() *
                _particles_width + connected.at( index ).x();

        for ( auto i = 0; i < neighbors.at( index ).count(); i++ )
        {
            auto index_neibor = connected.at( neighbors.at( index ).at(i) ).y() *
                    _particles_width +
                    connected.at( neighbors.at( index ).at(i) ).x();

            auto cond_1 = fabs( _heightvals.at( index_center ) -
                                _heightvals.at( index_neibor) ) < _smooth_thr;

            auto cond_2 = fabs( _particles[index_neibor].get_pos().f().at(1) -
                    _heightvals.at( index_neibor ) ) < _height_thr;

            if ( cond_1 && cond_2 )
            {
                auto offsetVec = Vec3( 0,
                                       _heightvals.at( index_neibor ) -
                                       _particles[index_neibor].get_pos().f().at(1),
                                       0 );
                _particles[index_neibor].offset_pos( offsetVec );
                _particles[index_neibor].make_unmovable();

                if ( !visited.at( neighbors.at( index ).at(i) ) )
                {
                    que.push( neighbors.at( index ).at(i) );
                    visited[ neighbors.at( index ).at(i) ] = true;
                }
            }
        }
    }
}
//=======================================================================================


//=======================================================================================
Particle * Cloth::get_particle( int x, int y )
{
    return & _particles[ y * _particles_width + x ];
}
//=======================================================================================
void Cloth::make_constraint( Particle* p1, Particle* p2 )
{
    p1->neighbors_list().push_back(p2);
    p2->neighbors_list().push_back(p1);
}
//=======================================================================================
int Cloth::get_size()
{
    return _particles_width * _particles_height;
}
//=======================================================================================
int Cloth::get_1D_index( const int x, const int y )
{
    return y * _particles_width + x;
}
//=======================================================================================
QVector<double> & Cloth::getHeightvals()
{
    return _heightvals;
}
//=======================================================================================
Particle * Cloth::getParticle1d( int index )
{
    return & _particles[ index ];
}
//=======================================================================================

