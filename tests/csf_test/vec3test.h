#ifndef VEC3TEST_H
#define VEC3TEST_H

#include "vec3.h"

#include "gtest/gtest.h"

//=======================================================================================
TEST( vec3_test_array, test_main )
{
    Vec3 vec( 0, 0, 0 );

    ASSERT_EQ( 3, vec.f().size() );

    for ( auto val: vec.f() )
        ASSERT_NEAR( 0.0, val, 0.001 );
}
//=======================================================================================
TEST( vec3_test_length, test_main )
{
    Vec3 vec { 0, 0, 0 };

    ASSERT_NEAR( 0.0, vec.length(), 0.001 );

    vec = { - 1, - 1, 0 };

    ASSERT_LE( 0.0, vec.length() );
}
//=======================================================================================
TEST( vec3_test_normilized, test_main )
{
    Vec3 vec { 15, 2, 1 };

    auto actual = vec.normalized();

    for ( auto val: actual.f() )
    {
        ASSERT_GE( val, 0.0 );
        ASSERT_LE( val, 1.0 );
    }
}
//=======================================================================================
TEST( vec3_test_cross, test_main )
{
    Vec3 vec { 2, 2, 2 };

    auto actual = vec.cross( { - 1, - 1, - 1 } );

    for ( auto val: actual.f() )
        ASSERT_GE( val, 0.0 );
}
//=======================================================================================
TEST( vec3_test_dot, test_main )
{
    Vec3 vec { 15, 2, 1 };

    auto actual = vec.dot( { - 1.0, - 2.0, - 3.0 } );

    ASSERT_LE( actual, 0.0 );
}
//=======================================================================================
TEST( vec3_test_operators, test_main )
{
    Vec3 vec { 2, 2, 2 };

    vec += { - 1, - 1, - 1 };

    for ( auto val: vec.f() )
        ASSERT_NEAR( val, 1.0, 0.001 );

    auto actual = vec / 2;

    for ( auto val: actual.f() )
        ASSERT_NEAR( val, 0.5, 0.001 );

    actual = actual * 2.0;

    for ( auto val: actual.f() )
        ASSERT_NEAR( val, 1.0, 0.001 );

    actual = actual - Vec3( 1, 1, 1 );

    for ( auto val: actual.f() )
        ASSERT_NEAR( val, 0.0, 0.001 );
}
//=======================================================================================

#endif // VEC3TEST_H
