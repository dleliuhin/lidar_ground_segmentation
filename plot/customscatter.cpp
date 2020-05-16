#include "customscatter.h"

using namespace QtDataVisualization;

//=======================================================================================
static QMap<int, QColor> colors
{
    { 0, QColor( Qt::black  ) },
    { 1, QColor( Qt::blue   ) },
    { 2, QColor( Qt::red    ) },
    { 3, QColor( Qt::green  ) },
    { 4, QColor( Qt::yellow ) },
    { 5, QColor( Qt::gray   ) }
};
//=======================================================================================


//=======================================================================================
CustomScatter::CustomScatter()
{
    this->setFlags( this->flags() ^ Qt::FramelessWindowHint );

    this->setPosition( { 550, 0 } );
    this->setWidth( 800 );
    this->setHeight( 700 );

    this->axisX()->setTitle("X");
    this->axisY()->setTitle("Y");
    this->axisZ()->setTitle("Z");

    this->axisX()->setRange( - 75, 75 );
    this->axisY()->setRange( 0, 150 );

    this->setShadowQuality( QAbstract3DGraph::ShadowQualityNone );

    this->scene()->activeCamera()->setCameraPreset( Q3DCamera::CameraPresetBehindHigh );
    this->scene()->activeCamera()->setCameraPosition( 0, 100, 450.0F );
    this->scene()->activeCamera()->setXRotation( 180 );
    this->scene()->activeCamera()->setYRotation( - 90 );

    this->show();
}
//=======================================================================================
CustomScatter::~CustomScatter()
{
    for ( const auto key: _layers.keys() )
        if ( _layers.value( key ) != nullptr )
            this->removeSeries( _layers[key] );

    this->close();
    this->destroy();
}
//=======================================================================================


//=======================================================================================
void CustomScatter::draw( const csf::PointCloud& data, int type )
{
    if ( data.empty() )
    {
        _replot();
        return;
    }

    for ( const auto& pnt: data )
    {
        if ( data.empty() )
            continue;

        auto tmp = new QScatter3DSeries;
        QScatterDataArray scatter_data;

        scatter_data << QVector3D( pnt.y, pnt.x, pnt.z );

        tmp->dataProxy()->addItems( scatter_data );
        tmp->setItemSize( 0.03F );
        tmp->setBaseColor( colors.value( type ) );
        tmp->setMesh( QAbstract3DSeries::MeshPoint );

        if ( type == 0 )
            tmp->setItemSize( 0.01F );

        this->addSeries( tmp );
    }

    _replot();
}
//=======================================================================================


//=======================================================================================
void CustomScatter::_replot()
{
    this->show();
}
//=======================================================================================
void CustomScatter::_clear_series()
{
    for ( auto& ser: this->seriesList() )
        this->removeSeries( ser );
}
//=======================================================================================
