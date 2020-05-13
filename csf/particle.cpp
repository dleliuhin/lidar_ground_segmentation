#include "particle.h"

//=======================================================================================
Particle::Particle(Vec3 pos, double time_step)
    : _movable            ( true            )
    , _mass               ( 1               )
    , _acceleration       ( Vec3( 0, 0, 0 ) )
    , _accumulated_normal ( Vec3( 0, 0, 0 ) )
    , _time_step_2        ( time_step       )
    , pos                 ( pos             )
    , old_pos             ( pos             )
{
    is_visited = false;
    neibor_count = 0;
    pos_x = 0;
    pos_y = 0;
    c_pos = 0;
    nearest_point_height = min_inf;
    tmp_dist = max_inf;
}
//=======================================================================================
Particle::Particle()
    : _movable            ( true            )
    , _mass               ( 1               )
    , _acceleration       ( Vec3( 0, 0, 0 ) )
    , _accumulated_normal ( Vec3( 0, 0, 0 ) )
    , pos                 ( Vec3( 0, 0, 0 ) )
    , old_pos             ( Vec3( 0, 0, 0 ) )
{
    is_visited = false;
    neibor_count = 0;
    pos_x = 0;
    pos_y = 0;
    c_pos = 0;
    nearest_point_height = min_inf;
    tmp_dist = max_inf;
}
//=======================================================================================

//=======================================================================================
void Particle::time_step()
{
    if ( _movable )
    {
        Vec3 temp = pos;
        pos = pos + ( pos - old_pos ) * ( 1.0 - damping ) + _acceleration * _time_step_2;
        old_pos = temp;
    }
}
//=======================================================================================
void Particle::satisfy_constraint_self( const int constraint_times )
{
    auto p1 = this;

    for ( auto i = 0; i < neighbors_list.size(); i++ )
    {
        auto p2 = neighbors_list[i];

        Vec3 correctionVector( 0, p2->pos.f[1] - p1->pos.f[1], 0 );

        if ( p1->is_movable() && p2->is_movable() )
        {
            auto correctionVectorHalf = correctionVector * (
                        constraint_times > 14 ? 0.5 : double_move_1[constraint_times]
                                               );
            p1->offset_pos( correctionVectorHalf );
            p2->offset_pos( - correctionVectorHalf );
        }

        else if ( p1->is_movable() && !p2->is_movable() )
        {
            auto correctionVectorHalf = correctionVector * (
                        constraint_times > 14 ? 1 : single_move_1[constraint_times]
                                               );
            p1->offset_pos( correctionVectorHalf );
        }

        else if ( !p1->is_movable() && p2->is_movable() )
        {
            auto correctionVectorHalf = correctionVector * (
                        constraint_times > 14 ? 1 : single_move_1[constraint_times]
                                               );
            p2->offset_pos( - correctionVectorHalf );
        }
    }
}
//=======================================================================================
