#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

//=======================================================================================
class Vec3
{
public:

    Vec3( double x, double y, double z )
        : _f ( { x, y, z } )
    {

    }

    Vec3() = default;

    //-----------------------------------------------------------------------------------

    const std::array<double, 3> & f() const
    {
        return _f;
    }

    //-----------------------------------------------------------------------------------

    double length()
    {
        return sqrt( pow( _f.at(0), 2 ) + pow( _f.at(1), 2 ) + pow( _f.at(2), 2 ) );
    }

    Vec3 normalized()
    {
        double l = length();

        return { _f.at(0) / l, _f.at(1) / l, _f.at(2) / l };
    }

    //-----------------------------------------------------------------------------------

    void operator +=( const Vec3& v )
    {
        _f[0] += v._f.at(0);
        _f[1] += v._f.at(1);
        _f[2] += v._f.at(2);
    }

    Vec3  operator /( const double a )
    {
        return { _f.at(0) / a, _f.at(1) / a, _f.at(2) / a };
    }

    Vec3 operator -( const Vec3& v )
    {
        return { _f.at(0) - v._f.at(0), _f.at(1) - v._f.at(1), _f.at(2) - v._f.at(2) };
    }

    Vec3 operator +( const Vec3& v )
    {
        return { _f.at(0) + v._f.at(0), _f.at(1) + v._f.at(1), _f.at(2) + v._f.at(2) };
    }

    Vec3 operator *( const double a )
    {
        return { _f.at(0) * a, _f.at(1) * a, _f.at(2) * a };
    }

    Vec3 operator -()
    {
        return { - _f.at(0), - _f.at(1), - _f.at(2) };
    }

    //-----------------------------------------------------------------------------------

    Vec3 cross( const Vec3& v )
    {
        return { _f[1] * v._f[2] - _f[2] * v._f[1],
                    _f[2] * v._f[0] - _f[0] * v._f[2],
                    _f[0] * v._f[1] - _f[1] * v._f[0] };
    }

    double dot( const Vec3& v )
    {
        return _f.at(0) * v._f.at(0) + _f.at(1) * v._f.at(1) + _f.at(2) * v._f.at(2);
    }

    //-----------------------------------------------------------------------------------

private:

    std::array<double, 3> _f;

};
//=======================================================================================

#endif // VEC3_H
