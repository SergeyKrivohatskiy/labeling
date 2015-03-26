#ifndef GEOM2_TO_QT
#define GEOM2_TO_QT
#include "geometry.h"
#include <QPoint>
#include <QRect>
#include <QLine>

namespace geom2
{
    QPoint to_qt(const point_i &p);

    QSize to_qt(const size_i &s);

    QRect to_qt(const rectangle_i &r);

    QLine to_qt(const segment_i &s);
} // namespace geom2
#endif // GEOM2_TO_QT
