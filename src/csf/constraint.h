#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "vec3.h"
#include "particle.h"
#include <map>
//=======================================================================================

//pre-computed constatnts by formula - (1-k)*0.4+k where k is previous constant

static QVector<double> single_move =
{
    0.4, (1-0.4)*0.4 + 0.4,
         (1-0.64)*0.4 + 0.64,
         (1-0.784)*0.4 + 0.784,
         0.92224, 0.95334, 0.97201, 0.9832, 0.98992,
         0.99395, 0.99637, 0.99782, 0.99869, 0.99922
};

static QVector<double> double_move =
{
    0.4, 0.48, 0.496, 0.4992, 0.49984, 0.49997,
    0.49999, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
};

//=======================================================================================
class Constraint
{
public:

    Constraint( Particle* p1, Particle* p2 );

    void satisfy_constraint( int constraint_times );

    //-----------------------------------------------------------------------------------

private:

    Particle *_p1, *_p2;

};
//=======================================================================================

#endif // CONSTRAINT_H
