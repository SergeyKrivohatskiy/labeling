#include "geom2_to_qt.h"

namespace geom2
{

    QPoint to_qt(const point_i &p)
    {
        return QPoint(p.x, p.y);
    }

    QSize to_qt(const size_i &s)
    {
        return QSize(s.w, s.h);
    }

    QRect to_qt(const rectangle_i &r)
    {
        return QRect(to_qt(r.left_bottom), to_qt(r.sz));
    }

    QLine to_qt(const segment_i &s)
    {
        return QLine(to_qt(s.start), to_qt(s.end));
    }
} // namespace geom2
