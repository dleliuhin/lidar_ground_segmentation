#ifndef CLOTH_H
#define CLOTH_H

#include "vec3.h"
#include "particle.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <omp.h>
#include <sstream>
#include <list>
#include <cmath>
#include <string>
#include <queue>

//=======================================================================================

static constexpr auto postprocess_max_particles = 50;

//=======================================================================================
struct XY
{
    XY( const int x1, const int y1 )
    {
        x = x1;
        y = y1;
    }

    int x;
    int y;
};
//=======================================================================================


//=======================================================================================
class Cloth
{
public:

    Vec3 origin_pos;
    double step_x;
    double step_y;
    std::vector<double> heightvals;
    int particles_width;
    int particles_height;

    //-----------------------------------------------------------------------------------

    Cloth( const Vec3& origin_pos,
           const int particles_width,
           const int particles_height,
           const double step_x,
           const double step_y,
           const double _smooth_thr,
           const double _height_thr,
           const int _rigidness,
           const double _time_step );

    //-----------------------------------------------------------------------------------

    double time_step();

    void add_force( const Vec3& direction );

    void terr_collision();

    void movable_filter();

    std::vector<int> find_unmovable( const std::vector<XY>& connected );

    void handle_slop_connected( const std::vector<int>& edge_points,
                                const std::vector<XY>& connected,
                                const std::vector<std::vector<int>>& neighbors );

    //-----------------------------------------------------------------------------------

    Particle * get_particle( const int x, const int y )
    {
        return & _particles[ y * particles_width + x ];
    }

    void make_constraint( Particle *p1, Particle *p2 )
    {
        p1->neighbors_list.push_back(p2);
        p2->neighbors_list.push_back(p1);
    }

    int get_size()
    {
        return particles_width * particles_height;
    }

    std::size_t get_1D_index( const int x, const int y )
    {
        return y * particles_width + x;
    }

    inline std::vector<double> & getHeightvals()
    {
        return heightvals;
    }

    Particle * getParticle1d( const int index )
    {
        return & _particles[ index ];
    }

    //-----------------------------------------------------------------------------------

private:

    int _constraint_iters;
    int _rigidness;
    double _time_step;

    std::vector<Particle> _particles;

    double _smooth_thr;
    double _height_thr;

};
//=======================================================================================

#endif // CLOTH_H
