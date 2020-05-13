#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec3.h"
#include <vector>

//=======================================================================================
static constexpr auto damping = 0.01;
static constexpr auto max_inf = 9999999999;
static constexpr auto min_inf = - 9999999999;

static constexpr double single_move_1[15] =
{
    0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765,
    0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322
};

static constexpr double double_move_1[15] =
{
    0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498,
    0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5
};

//=======================================================================================
class Particle
{
public:

    Vec3 pos;
    Vec3 old_pos;
    bool is_visited;
    int neibor_count;
    int pos_x;
    int pos_y;
    int c_pos;

    std::vector<Particle*> neighbors_list;
    std::vector<int> corresponding_point_list;
    std::size_t nearest_point_id;

    double nearest_point_height;
    double tmp_dist;

    //-----------------------------------------------------------------------------------

    Particle( Vec3 pos, double time_step );
    Particle();

    //-----------------------------------------------------------------------------------

    void satisfy_constraint_self( const int constraintTimes );

    bool is_movable()
    {
        return _movable;
    }

    void add_force( Vec3 f )
    {
        _acceleration += f / _mass;
    }

    void time_step();

    Vec3& get_pos()
    {
        return pos;
    }

    Vec3 get_pos_copy()
    {
        return pos;
    }

    void reset_acceleration()
    {
        _acceleration = Vec3( 0, 0, 0 );
    }

    void offset_pos( const Vec3& v )
    {
        if ( _movable ) pos += v;
    }

    void make_unmovable()
    {
        _movable = false;
    }

    void add_to_normal( Vec3 normal )
    {
        _accumulated_normal += normal.normalized();
    }

    Vec3& get_normal()
    {
        return _accumulated_normal;
    }

    void reset_normal()
    {
        _accumulated_normal = Vec3( 0, 0, 0 );
    }

    //-----------------------------------------------------------------------------------

private:

    bool _movable;
    double _mass;
    Vec3 _acceleration;
    Vec3 _accumulated_normal;
    double _time_step_2;
};
//=======================================================================================

#endif // PARTICLE_H
