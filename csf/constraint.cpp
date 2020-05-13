#include "constraint.h"

//=======================================================================================
void Constraint::satisfy_constraint( const int constraint_times )
{
    Vec3 correctionVector( 0, p2->pos.f[1] - p1->pos.f[1], 0 );

    if ( p1->is_movable() && p2->is_movable() )
    {
        auto correctionVectorHalf = correctionVector * (
            constraint_times > 14 ? 0.5 : double_move[ constraint_times - 1 ]
        );

        p1->offset_pos( correctionVectorHalf );
        p2->offset_pos( - correctionVectorHalf );
    }

    else if (p1->is_movable() && !p2->is_movable())
    {
        auto correctionVectorHalf = correctionVector * (
            constraint_times > 14 ? 1 : single_move[ constraint_times - 1 ]
        );

        p1->offset_pos( correctionVectorHalf );
    }

    else if ( !p1->is_movable() && p2->is_movable() )
    {
        auto correctionVectorHalf = correctionVector * (
            constraint_times > 14 ? 1 : single_move[ constraint_times - 1 ]
        );

        p2->offset_pos( - correctionVectorHalf );
    }
}
//=======================================================================================
