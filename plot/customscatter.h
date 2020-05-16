#ifndef CUSTOMSCATTER_H
#define CUSTOMSCATTER_H

#include "point_cloud.h"

#include <QColor>
#include <Q3DScatter>
#include <QWidget>
#include <QObject>
#include <QList>

using namespace QtDataVisualization;

//=======================================================================================
class CustomScatter : public Q3DScatter
{
    Q_OBJECT

public:

    CustomScatter();
    ~CustomScatter() override;

    //-------------------------------------------------------------------------------

public slots:

    void draw( const csf::PointCloud& data, int type );

    //-------------------------------------------------------------------------------

private:

    QMap<int, QScatter3DSeries*> _layers;

    //-----------------------------------------------------------------------------------

    void _replot();

    void _clear_series();
};

//=======================================================================================

#endif // CUSTOMSCATTER_H
