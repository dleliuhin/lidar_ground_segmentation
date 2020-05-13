#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QFile>
#include <QList>

#include "csf.h"

using VecInt = std::vector<int>;

//=======================================================================================
int main( int argc, char **argv )
{
    QCoreApplication qapp( argc, argv );

    //-----------------------------------------------------------------------------------

    auto fname = "../lidar_ground_segmentation/dataset.txt";
    QFile jfile( fname );
    jfile.open( QFile::ReadOnly );

    auto doc = QJsonDocument().fromJson( jfile.readAll() );

    //-----------------------------------------------------------------------------------

    auto data = doc.object();

    csf::PointCloud cloud;

    for ( const auto& pnt: data )
    {
        csf::Point tmp;

        tmp.x = pnt.toObject().value("x").toDouble();
        tmp.y = pnt.toObject().value("y").toDouble();
        tmp.z = pnt.toObject().value("z").toDouble();

        cloud.push_back( tmp );
    }

    //-----------------------------------------------------------------------------------

    CSF csfilter;

    csfilter.setPointCloud( cloud );

    csfilter.params.sloop_smooth     = true;
    csfilter.params.time_step        = 0.8;
    csfilter.params.class_thr        = 0.35;
    csfilter.params.cloth_resolution = 1.0;
    csfilter.params.iterations       = 30;
    csfilter.params.rigidness        = 1.0;

    VecInt ground;
    VecInt nonground;

    csfilter.split( ground, nonground );

    //-----------------------------------------------------------------------------------

    return qapp.exec();
}
//=======================================================================================
