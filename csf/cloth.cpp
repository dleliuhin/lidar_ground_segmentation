#include "cloth.h"

//=======================================================================================
Cloth::Cloth( const Vec3& origin_pos,
              const int pwidth,
              const int pheight,
              const double step_x,
              const double step_y,
              const double smooth_threshold,
              const double height_threshold,
              const int rigidness,
              const double time_step )
    : origin_pos        ( origin_pos       )
    , step_x            ( step_x           )
    , step_y            ( step_y           )
    , particles_width   ( pwidth           )
    , particles_height  ( pheight          )
    , _constraint_iters ( rigidness        )
    , _time_step        ( time_step        )
    , _smooth_thr       ( smooth_threshold )
    , _height_thr       ( height_threshold )
{
    _particles.resize( size_t( pwidth * pheight ) );

    auto time_step_2 = pow( time_step, 2 );

    for ( auto i = 0; i < pwidth; i++ )
        for ( auto j = 0; j < pheight; j++ )
        {
            Vec3 pos( origin_pos.f[0] + i * step_x,
                    origin_pos.f[1],
                    origin_pos.f[2] + j * step_y );

            _particles[ j * pwidth + i ] = Particle( pos, time_step_2 );
            _particles[ j * pwidth + i ].pos_x = i;
            _particles[ j * pwidth + i ].pos_y = j;
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
double Cloth::time_step()
{
    for ( size_t i = 0; i < _particles.size(); i++ )
        _particles[i].time_step();

    for ( size_t i = 0; i < _particles.size(); i++ )
        _particles[i].satisfy_constraint_self( _constraint_iters );

    auto max_diff = 0.0;

    for ( size_t i = 0; i < _particles.size(); i++ )
        if ( _particles[i].is_movable() )
        {
            auto diff = fabs( _particles[i].old_pos.f[1] - _particles[i].pos.f[1] );

            if ( diff > max_diff )
                max_diff = diff;
        }

    return max_diff;
}
//=======================================================================================
void Cloth::add_force( const Vec3& direction )
{
    for ( auto i = 0; i < _particles.size(); i++ )
        _particles[i].add_force( direction );
}
//=======================================================================================
void Cloth::terr_collision()
{
    for ( auto i = 0; i < _particles.size(); i++ )
    {
        Vec3 v = _particles[i].get_pos();

        if ( v.f[1] < heightvals[i] )
        {
            _particles[i].offset_pos( Vec3( 0, heightvals[i] - v.f[1], 0 ) );
            _particles[i].make_unmovable();
        }
    }
}
//=======================================================================================
void Cloth::movable_filter()
{
    std::vector<Particle> tmpParticles;

    for ( int x = 0; x < particles_width; x++ )
        for ( int y = 0; y < particles_height; y++ )
        {
            auto ptc = get_particle( x, y );

            if ( ptc->is_movable() && !ptc->is_visited )
            {
                std::queue<int> que;
                std::vector<XY> connected;
                std::vector<std::vector<int>> neighbors;
                auto sum = 1;
                auto index = y * particles_width + x;

                connected.push_back( XY( x, y ) );
                _particles[ index ].is_visited = true;

                que.push( index );

                while ( !que.empty() )
                {
                    Particle *ptc_f = &_particles[ que.front() ];
                    que.pop();
                    auto cur_x = ptc_f->pos_x;
                    auto cur_y = ptc_f->pos_y;
                    std::vector<int> neighbor;

                    if ( cur_x > 0 )
                    {
                        auto ptc_left = get_particle(cur_x - 1, cur_y);

                        if ( ptc_left->is_movable() )
                        {
                            if ( !ptc_left->is_visited )
                            {
                                sum++;
                                ptc_left->is_visited = true;
                                connected.push_back( XY( cur_x - 1, cur_y ) );
                                que.push( particles_width * cur_y + cur_x - 1 );
                                neighbor.push_back( sum - 1 );
                                ptc_left->c_pos = sum - 1;
                            }

                            else
                                neighbor.push_back(ptc_left->c_pos);
                        }
                    }

                    if ( cur_x < particles_width - 1 )
                    {
                        auto ptc_right = get_particle( cur_x + 1, cur_y );

                        if ( ptc_right->is_movable() )
                        {
                            if ( !ptc_right->is_visited )
                            {
                                sum++;
                                ptc_right->is_visited = true;
                                connected.push_back( XY( cur_x + 1, cur_y ) );
                                que.push( particles_width * cur_y + cur_x + 1 );
                                neighbor.push_back( sum - 1 );
                                ptc_right->c_pos = sum - 1;
                            }

                            else
                                neighbor.push_back( ptc_right->c_pos );
                        }
                    }

                    if ( cur_y > 0 )
                    {
                        auto ptc_bottom = get_particle( cur_x, cur_y - 1 );

                        if ( ptc_bottom->is_movable() )
                        {
                            if ( !ptc_bottom->is_visited )
                            {
                                sum++;
                                ptc_bottom->is_visited = true;
                                connected.push_back( XY( cur_x, cur_y - 1 ) );
                                que.push( particles_width * ( cur_y - 1 ) + cur_x );
                                neighbor.push_back( sum - 1 );
                                ptc_bottom->c_pos = sum - 1;
                            }

                            else
                                neighbor.push_back(ptc_bottom->c_pos);
                        }
                    }

                    if ( cur_y < particles_height - 1 )
                    {
                        auto ptc_top = get_particle(cur_x, cur_y + 1);

                        if ( ptc_top->is_movable() )
                        {
                            if ( !ptc_top->is_visited )
                            {
                                sum++;
                                ptc_top->is_visited = true;
                                connected.push_back( XY( cur_x, cur_y + 1 ) );
                                que.push( particles_width * ( cur_y + 1 ) + cur_x );
                                neighbor.push_back( sum - 1 );
                                ptc_top->c_pos = sum - 1;
                            }

                            else
                                neighbor.push_back( ptc_top->c_pos );
                        }
                    }
                    neighbors.push_back( neighbor );
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
std::vector<int> Cloth::find_unmovable( const std::vector<XY>& connected )
{
    std::vector<int> edge_points;

    for ( auto i = 0; i < connected.size(); i++ )
    {
        auto x = connected.at(i).x;
        auto y = connected.at(i).y;
        auto index = y * particles_width + x;

        auto ptc = get_particle( x, y );

        if ( x > 0 )
        {
            auto ptc_x = get_particle( x - 1, y );

            if ( !ptc_x->is_movable() )
            {
                auto index_ref = y * particles_width + x - 1;

                auto cond_1 = fabs( heightvals.at( index ) -
                                    heightvals.at( index_ref ) ) < _smooth_thr;
                auto cond_2 = ptc->get_pos().f[1] - heightvals.at( index ) < _height_thr;

                if ( cond_1 && cond_2 )
                {
                    auto offset_vec = Vec3( 0,
                                            heightvals[index] - ptc->get_pos().f[1], 0 );
                    _particles[ index ].offset_pos( offset_vec );
                    ptc->make_unmovable();
                    edge_points.push_back(i);

                    continue;
                }
            }
        }

        if ( x < particles_width - 1 )
        {
            auto ptc_x = get_particle( x + 1, y );

            if ( !ptc_x->is_movable() )
            {
                auto index_ref = y * particles_width + x + 1;

                auto cond_1 = fabs( heightvals[index] -
                                    heightvals[index_ref] ) < _smooth_thr;

                auto cond_2 = ptc->get_pos().f[1] - heightvals[index] < _height_thr;

                if ( cond_1 && cond_2 )
                {
                    auto offsetVec = Vec3( 0,
                                           heightvals[index] - ptc->get_pos().f[1], 0 );
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
                auto index_ref = ( y - 1 ) * particles_width + x;

                auto cond_1 = fabs( heightvals[index] -
                                    heightvals[index_ref] ) < _smooth_thr;

                auto cond_2 = ( ptc->get_pos().f[1] - heightvals[index] < _height_thr );

                if ( cond_1 && cond_2 )
                {
                    auto offsetVec = Vec3( 0,
                                           heightvals[index] - ptc->get_pos().f[1], 0 );
                    _particles[index].offset_pos( offsetVec );
                    ptc->make_unmovable();
                    edge_points.push_back(i);

                    continue;
                }
            }
        }

        if ( y < particles_height - 1 )
        {
            auto ptc_y = get_particle( x, y + 1 );

            if ( !ptc_y->is_movable() )
            {
                auto index_ref = ( y + 1 ) * particles_width + x;

                auto cond_1 = fabs( heightvals[index] -
                                    heightvals[index_ref] ) < _smooth_thr;

                auto cond_2 = ( ptc->get_pos().f[1] - heightvals[index] < _height_thr );

                if ( cond_1 && cond_2 )
                {
                    auto offsetVec = Vec3( 0,
                                           heightvals[index] - ptc->get_pos().f[1], 0 );
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
void Cloth::handle_slop_connected( const std::vector<int>& edge_points,
                                   const std::vector<XY>& connected,
                                   const std::vector<std::vector<int>>& neighbors )
{
    std::vector<bool> visited;

    for ( size_t i = 0; i < connected.size(); i++ )
        visited.push_back( false );

    std::queue<int> que;

    for ( size_t i = 0; i < edge_points.size(); i++ )
    {
        que.push( edge_points[i] );
        visited[edge_points[i]] = true;
    }

    while ( !que.empty() )
    {
        auto index = que.front();
        que.pop();

        auto index_center = connected[index].y * particles_width + connected[index].x;

        for ( size_t i = 0; i < neighbors[index].size(); i++ )
        {
            auto index_neibor = connected[neighbors[index][i]].y *
                    particles_width +
                    connected[neighbors[index][i]].x;

            auto cond_1 = fabs( heightvals[index_center] -
                                heightvals[index_neibor] ) < _smooth_thr;

            auto cond_2 = fabs( _particles[index_neibor].get_pos().f[1] -
                    heightvals[index_neibor] ) < _height_thr;

            if ( cond_1 && cond_2 )
            {
                auto offsetVec = Vec3( 0,
                                       heightvals[index_neibor] -
                                       _particles[index_neibor].get_pos().f[1], 0 );
                _particles[index_neibor].offset_pos( offsetVec );
                _particles[index_neibor].make_unmovable();

                if ( visited[neighbors[index][i]] == false )
                {
                    que.push(neighbors[index][i]);
                    visited[neighbors[index][i]] = true;
                }
            }
        }
    }
}
//=======================================================================================
