#ifndef CLOTH_TEST_H
#define CLOTH_TEST_H

#include "cloth.h"

#include "gtest/gtest.h"

//=======================================================================================
TEST( cloth_test_xy, test_main )
{
    XY actual( 0, 0 );

    ASSERT_EQ( actual.x(), 0 );
    ASSERT_EQ( actual.y(), 0 );
}
//=======================================================================================
TEST( cloth_test_getters, test_main )
{
    Vec3 vec{ 0, 0, 0 };
    Cloth actual { vec, 10, 10, 0, 0, 0, 0, 0, 0 };

    for ( size_t i = 0; i < vec.f().size(); ++i )
        ASSERT_NEAR( actual.origin_pos().f().at(i), vec.f().at(i), 0.001 );

    ASSERT_NEAR( 10, actual.particles_width(), 0.001 );
    ASSERT_NEAR( 10, actual.particles_height(), 0.001 );
    ASSERT_NEAR( 0, actual.step_x(), 0.001 );
    ASSERT_NEAR( 0, actual.step_y(), 0.001 );
    ASSERT_NEAR( 0, actual.time_step(), 0.001 );
}
//=======================================================================================
TEST( cloth_test_size, test_main )
{
    Vec3 vec{ 0, 0, 0 };
    Cloth actual { vec, 10, 10, 0, 0, 0, 0, 0, 0 };

    ASSERT_EQ( actual.get_size(), 100 );
}
//=======================================================================================
TEST( cloth_test_index, test_main )
{
    Vec3 vec{ 0, 0, 0 };
    Cloth actual { vec, 10, 10, 0, 0, 0, 0, 0, 0 };

    ASSERT_EQ( actual.get_1D_index( 0, 1 ), 10 );
}
//=======================================================================================
TEST( cloth_test_get_particle, test_main )
{
    Vec3 vec{ 0, 0, 0 };
    Cloth actual { vec, 10, 10, 0, 0, 0, 0, 0, 0 };

    auto p = actual.get_particle( 0, 0 );

    ASSERT_EQ( p->pos_x(), 0 );
    ASSERT_EQ( p->pos_y(), 0 );
    ASSERT_EQ( p->is_movable(), true );
    ASSERT_EQ( actual.getHeightvals().size(), 0 );
}
//=======================================================================================
TEST( cloth_test_getParticle1d, test_main )
{
    Vec3 vec{ 0, 0, 0 };
    Cloth actual { vec, 10, 10, 0, 0, 0, 0, 0, 0 };

    auto p = actual.getParticle1d(0);

    ASSERT_EQ( p->pos_x(), 0 );
    ASSERT_EQ( p->pos_y(), 0 );
    ASSERT_EQ( p->is_movable(), true );
}
//=======================================================================================
TEST( cloth_test_make_constraint, test_main )
{
    Vec3 vec{ 0, 0, 0 };
    Cloth actual { vec, 10, 10, 0, 0, 0, 0, 0, 0 };

    Particle p1( vec, 0 );
    vec = { 1, 0, 1 };
    Particle p2( vec, 0 );

    actual.make_constraint( &p1, &p2 );

    ASSERT_EQ( actual.get_size(), 100 );
}
//=======================================================================================

#endif // CLOTH_TEST_H
