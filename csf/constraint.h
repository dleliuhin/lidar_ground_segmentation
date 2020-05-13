#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "vec3.h"
#include "particle.h"

//=======================================================================================
static constexpr double single_move[14] =
{
    0.4, 0.64, 0.784, 0.8704, 0.92224, 0.95334, 0.97201,
    0.9832, 0.98992, 0.99395, 0.99637, 0.99782, 0.99869, 0.99922
};

static constexpr double double_move[14] =
{
    0.4, 0.48, 0.496, 0.4992, 0.49984, 0.49997,
    0.49999, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
};

//=======================================================================================
class Constraint
{
public:

    Particle *p1, *p2;

    Constraint( Particle* p1, Particle* p2 )
        : p1 (p1)
        , p2 (p2)
    {}

    void satisfy_constraint( const int constraint_times );

    //-----------------------------------------------------------------------------------

private:

    double _rest_distance;

};
//=======================================================================================

#endif // CONSTRAINT_H
