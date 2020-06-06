#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "vec3.h"
#include "particle.h"
#include <map>
//=======================================================================================

//pre-computed constatnts by formula - (1-k)*0.4+k where k is previous constant

namespace single_rigid
{
    const double RI_1( 0.4 );
    const double RI_2( 0.64 );
    const double RI_3( 0.784 );
    const double RI_4( 0.8704 );
    const double RI_5( 0.92224 );
    const double RI_6( 0.95334 );
    const double RI_7( 0.97201 );
    const double RI_8( 0.9832 );
    const double RI_9( 0.98992 );
    const double RI_10( 0.99395 );
    const double RI_11( 0.99637 );
    const double RI_12( 0.99782 );
    const double RI_13( 0.99869 );
    const double RI_14( 0.99922 );
};

namespace double_rigid
{
    const double RI_1( 0.4 );
    const double RI_2( 0.48 );
    const double RI_3( 0.496 );
    const double RI_4( 0.4992 );
    const double RI_5( 0.49984 );
    const double RI_6( 0.49997 );
    const double RI_7( 0.49999 );
    const double RI_8( 0.5 );
    const double RI_9( 0.5 );
    const double RI_10( 0.5 );
    const double RI_11( 0.5 );
    const double RI_12( 0.5 );
    const double RI_13( 0.5 );
    const double RI_14( 0.5 );
};

static std::map<int, double> single_constraints = { { 1, single_rigid::RI_1 },
                                                       { 2, single_rigid::RI_2 },
                                                       { 3, single_rigid::RI_3 },
                                                       { 4, single_rigid::RI_4 },
                                                       { 5, single_rigid::RI_5 },
                                                       { 6, single_rigid::RI_6 },
                                                       { 7, single_rigid::RI_7 },
                                                       { 8, single_rigid::RI_8 },
                                                       { 9, single_rigid::RI_9 },
                                                       { 10, single_rigid::RI_10 },
                                                       { 11, single_rigid::RI_11 },
                                                       { 12, single_rigid::RI_12 },
                                                       { 13, single_rigid::RI_13 },
                                                       { 14, single_rigid::RI_14 } };

static std::map<int, double> double_constraints = { { 1, double_rigid::RI_1 },
                                                       { 2, double_rigid::RI_2 },
                                                       { 3, double_rigid::RI_3 },
                                                       { 4, double_rigid::RI_4 },
                                                       { 5, double_rigid::RI_5 },
                                                       { 6, double_rigid::RI_6 },
                                                       { 7, double_rigid::RI_7 },
                                                       { 8, double_rigid::RI_8 },
                                                       { 9, double_rigid::RI_9 },
                                                       { 10, double_rigid::RI_10 },
                                                       { 11, double_rigid::RI_11 },
                                                       { 12, double_rigid::RI_12 },
                                                       { 13, double_rigid::RI_13 },
                                                       { 14, double_rigid::RI_14 } };

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
