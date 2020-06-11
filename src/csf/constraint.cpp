#include "constraint.h"

static constexpr auto half = 0.5;
static constexpr auto one = 1;

//=======================================================================================
Constraint::Constraint(Particle *p1, Particle *p2)
    : _p1 (p1)
    , _p2 (p2)
{

}
//=======================================================================================


//=======================================================================================
void Constraint::satisfy_constraint( int constraint_times )
{     
    Vec3 correctionVector( 0, _p2->pos().f().at(1) - _p1->pos().f().at(1), 0 );

    if ( _p1->is_movable() && _p2->is_movable() )
    {
        auto correctionVectorHalf = correctionVector * (
            constraint_times > double_move.size() ? half : double_move.at( constraint_times - 1 )
        );

        _p1->offset_pos( correctionVectorHalf );
        _p2->offset_pos( - correctionVectorHalf );
    }

    else if ( _p1->is_movable() && !_p2->is_movable() )
    {
        auto correctionVectorHalf = correctionVector * (
            constraint_times > single_move.size() ? one : single_move.at( constraint_times - 1 )
        );

        _p1->offset_pos( correctionVectorHalf );
    }

    else if ( !_p1->is_movable() && _p2->is_movable() )
    {
        auto correctionVectorHalf = correctionVector * (
            constraint_times > single_move.size() ? one : single_move.at( constraint_times - 1 )
        );

        _p2->offset_pos( - correctionVectorHalf );
    }
}
//=======================================================================================
