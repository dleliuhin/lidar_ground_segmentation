#ifdef GUI
#include <QApplication>
#else
#include <QCoreApplication>
#endif
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QFile>
#include <QList>

#include "csf.h"
#ifdef GUI
#include "customscatter.h"
#endif

using VecInt = QVector<int>;

//=======================================================================================
int main( int argc, char **argv )
{
#ifdef GUI
    QApplication qapp( argc, argv );
#else
    QCoreApplication qapp( argc, argv );
#endif

    //-----------------------------------------------------------------------------------

    auto fname = "../data/dataset.txt";
    QFile jfile( fname );
    jfile.open( QFile::ReadOnly );

    auto doc = QJsonDocument::fromJson( jfile.readAll() );

    //-----------------------------------------------------------------------------------

    auto data = doc.object();

    csf::PointCloud cloud;

    for ( const auto pnt: data )
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

    Params params;

    params.sloop_smooth     = true;
    params.time_step        = 0.8;
    params.class_thr        = 0.35;
    params.cloth_resolution = 1.0;
    params.iterations       = 30;
    params.rigidness        = 1.0;

    csfilter.params( params );

    VecInt ground;
    VecInt nonground;

    csfilter.split( ground, nonground );

    //-----------------------------------------------------------------------------------

    csf::PointCloud ground_points;
    csf::PointCloud nonground_points;

    for ( const auto id: ground )
        ground_points.push_back( cloud.at(id) );

    for ( const auto id: nonground )
        nonground_points.push_back( cloud.at(id) );

    //-----------------------------------------------------------------------------------

#ifdef GUI

    auto scatter = new CustomScatter();
    scatter->draw( ground_points, 0 );
    scatter->draw( nonground_points, 1 );

#endif

    //-----------------------------------------------------------------------------------

    return qapp.exec();
}
//=======================================================================================
