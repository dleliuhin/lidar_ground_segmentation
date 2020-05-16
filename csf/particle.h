#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec3.h"

#include <QVector>

//=======================================================================================
static constexpr auto damping = 0.01;
static constexpr auto max_inf = 9999999999;
static constexpr auto min_inf = - 9999999999;

static constexpr std::array<double, 15> single_move_1 =
{
    0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765,
    0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322
};

static constexpr std::array<double, 15> double_move_1 =
{
    0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498,
    0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5
};

//=======================================================================================
class Particle
{
public:

    Particle( Vec3& _pos, double time_step );
    Particle() = default;

    //-----------------------------------------------------------------------------------

    const Vec3 & pos() const;
    const Vec3 & old_pos() const;
    int pos_x() const;
    void pos_x( int val );
    int pos_y() const;
    void pos_y( int val );
    double nearest_point_height() const;
    QVector<Particle*> & neighbors_list();
    void is_visited( bool val );
    bool is_visited() const;
    QVector<int> & corresponding_point_list();
    void tmp_dist( double val );
    double tmp_dist() const;
    void nearest_point_height( double val );
    void nearest_point_id( int val );
    void c_pos( int val );
    int c_pos() const;

    //-----------------------------------------------------------------------------------

    void satisfy_constraint_self( int constraint_times );

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
        return _pos;
    }

    Vec3 get_pos_copy()
    {
        return _pos;
    }

    void reset_acceleration()
    {
        _acceleration = Vec3( 0, 0, 0 );
    }

    void offset_pos( const Vec3& v )
    {
        if ( _movable ) _pos += v;
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

    bool _movable { true };
    double _mass {1};
    Vec3 _acceleration { 0, 0, 0 };
    Vec3 _accumulated_normal { 0, 0, 0 };
    double _time_step_2 {0};

    Vec3 _pos { 0, 0, 0 };
    Vec3 _old_pos { 0, 0, 0 };
    bool _is_visited { false };
    int _neighbour_count {0};
    int _pos_x {0};
    int _pos_y {0};
    int _c_pos {0};

    QVector<Particle*> _neighbors_list;
    QVector<int> _corresponding_point_list;
    int _nearest_point_id {0};

    double _nearest_point_height { min_inf };
    double _tmp_dist { max_inf };
};
//=======================================================================================

#endif // PARTICLE_H
