#ifndef CLOTH_H
#define CLOTH_H

#include "vec3.h"
#include "particle.h"

#include <QVector>

#include <cmath>


//=======================================================================================
class XY
{
public:

    XY( int x, int y );

    //-----------------------------------------------------------------------------------

    int x() const;
    int y() const;

    //-----------------------------------------------------------------------------------

private:

    int _x;
    int _y;
};
//=======================================================================================


//=======================================================================================
class Cloth
{
public:

    //-----------------------------------------------------------------------------------

    Cloth( Vec3& origin_pos,
           int pwidth,
           int pheight,
           double step_x,
           double step_y,
           double smooth_thr,
           double height_thr,
           int rigidness,
           double time_step );

    //-----------------------------------------------------------------------------------

    const Vec3 & origin_pos() const;
    double step_x() const;
    double step_y() const;
    int particles_width() const;
    int particles_height() const;

    //-----------------------------------------------------------------------------------

    double time_step();

    void add_force( const Vec3& direction );

    void terr_collision();

    void movable_filter();

    QVector<int> find_unmovable( const QVector<XY>& connected );

    void handle_slop_connected( const QVector<int>& edge_points,
                                const QVector<XY>& connected,
                                const QVector<QVector<int>>& neighbors );

    //-----------------------------------------------------------------------------------

    Particle * get_particle( int x, int y );

    void make_constraint( Particle* p1, Particle* p2 );

    int get_size();

    int get_1D_index( int x, int y );

    QVector<double> & getHeightvals();

    Particle * getParticle1d( int index );

    //-----------------------------------------------------------------------------------

private:

    Vec3 _origin_pos;
    double _step_x;
    double _step_y;
    QVector<double> _heightvals;
    int _particles_width;
    int _particles_height;

    int _constraint_iters;

    QVector<Particle> _particles;

    double _smooth_thr;
    double _height_thr;

};
//=======================================================================================

#endif // CLOTH_H
