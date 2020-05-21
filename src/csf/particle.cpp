#include "particle.h"

//=======================================================================================
Particle::Particle( Vec3& pos, double time_step )
    : _pos                ( pos       )
    , _old_pos            ( pos       )
    , _time_step_2        ( time_step )
{

}
//=======================================================================================


//=======================================================================================
const Vec3 & Particle::pos() const
{
    return _pos;
}
//=======================================================================================
const Vec3 &Particle::old_pos() const
{
    return _old_pos;
}
//=======================================================================================
int Particle::pos_x() const
{
    return _pos_x;
}
//=======================================================================================
void Particle::pos_x( int val )
{
    _pos_x = val;
}
//=======================================================================================
int Particle::pos_y() const
{
    return _pos_y;
}
//=======================================================================================
void Particle::pos_y( int val )
{
    _pos_y = val;
}
//=======================================================================================
double Particle::nearest_point_height() const
{
    return _nearest_point_height;
}
//=======================================================================================
QVector<Particle *> & Particle::neighbors_list()
{
    return _neighbors_list;
}
//=======================================================================================
void Particle::is_visited( bool val )
{
    _is_visited = val;
}
//=======================================================================================
bool Particle::is_visited() const
{
    return _is_visited;
}
//=======================================================================================
QVector<int> & Particle::corresponding_point_list()
{
    return _corresponding_point_list;
}
//=======================================================================================
void Particle::tmp_dist( double val )
{
    _tmp_dist = val;
}
//=======================================================================================
double Particle::tmp_dist() const
{
    return _tmp_dist;
}
//=======================================================================================
void Particle::nearest_point_height( double val )
{
    _nearest_point_height = val;
}
//=======================================================================================
void Particle::nearest_point_id( int val )
{
    _nearest_point_id = val;
}
//=======================================================================================
void Particle::c_pos( int val )
{
    _c_pos = val;
}
//=======================================================================================
int Particle::c_pos() const
{
    return _c_pos;
}
//=======================================================================================


//=======================================================================================
void Particle::time_step()
{
    if ( _movable )
    {
        Vec3 temp = _pos;
        _pos = _pos + ( _pos - _old_pos ) * damping + _acceleration * _time_step_2;
        _old_pos = temp;
    }
}
//=======================================================================================
void Particle::satisfy_constraint_self( int constraint_times )
{
    auto p1 = this;

    for ( const auto& p2: _neighbors_list )
    {
        Vec3 correction_vec = { 0, p2->pos().f().at(1) - p1->_pos.f().at(1), 0 };

        if ( p1->is_movable() && p2->is_movable() )
        {
            auto correction_vec_half = correction_vec * (
                        constraint_times > 14 ? 0.5 : double_move_1.at( constraint_times )
                                               );
            p1->offset_pos( correction_vec_half );
            p2->offset_pos( - correction_vec_half );

            continue;
        }

        auto correction_vec_half = correction_vec * (
                    constraint_times > 14 ? 1 : single_move_1.at( constraint_times ) );

        if ( p1->is_movable() && !p2->is_movable() )
            p1->offset_pos( correction_vec_half );

        else if ( !p1->is_movable() && p2->is_movable() )
            p2->offset_pos( - correction_vec_half );
    }
}
//=======================================================================================
