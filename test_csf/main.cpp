#include "gtest/gtest.h"

#include <iostream>
#include "clusterization.h"

#include "vlog.h"

#include "defs.h"
#include "config.h"

using namespace std;

//=======================================================================================
TEST( bounding_box_test, test_main )
{
    LidarData scan;
}
//=======================================================================================


//=======================================================================================
int main( int argc, char **argv )
{
    vdeb << "Program testing is started....";

    ::testing::InitGoogleTest( &argc, argv );

    return RUN_ALL_TESTS();
}
//=======================================================================================
