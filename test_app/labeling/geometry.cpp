#include "geometry.h"

namespace geom2
{

    const static int INSIDE = 0; // 0000
    const static int LEFT = 1;   // 0001
    const static int RIGHT = 2;  // 0010
    const static int BOTTOM = 4; // 0100
    const static int TOP = 8;    // 1000

    static int out_code(int x, int y, int xmin, int xmax, int ymin, int ymax)
    {
        int code = INSIDE;          // initialised as being inside of clip window

        if (x < xmin)           // to the left of clip window
            code |= LEFT;
        else if (x > xmax)      // to the right of clip window
            code |= RIGHT;
        if (y < ymin)           // below the clip window
            code |= BOTTOM;
        else if (y > ymax)      // above the clip window
            code |= TOP;

        return code;
    }

    /*
     * From
     * https://en.wikipedia.org/wiki/Cohen–Sutherland_algorithm
     */
    static int c_s_seg_rect_intersection(
            int x0, int y0, int x1, int y1,
            int xmin, int xmax, int ymin, int ymax,
            point_i *intersection_point1 = nullptr,
            point_i *intersection_point2 = nullptr,
            double *t1_out = nullptr,
            double *t2_out = nullptr)
    {
        // compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
        int outcode0 = out_code(x0, y0, xmin, xmax, ymin, ymax);
        int outcode1 = out_code(x1, y1, xmin, xmax, ymin, ymax);
        int intersections = 0;

        while (true) {
            if (!(outcode0 | outcode1)) { // Bitwise OR is 0. Trivially accept and get out of loop
                break;
            } else if (outcode0 & outcode1) { // Bitwise AND is not 0. Trivially reject and get out of loop
                break;
            } else {
                // failed both tests, so calculate the line segment to clip
                // from an outside point to an intersection with clip edge
                int x, y;

                // At least one endpoint is outside the clip rectangle; pick it.
                int outcodeOut = outcode0 ? outcode0 : outcode1;
                double t;

                // Now find the intersection point;
                // use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)
                if (outcodeOut & TOP) {           // point is above the clip rectangle
                    t = (ymax - y0) / (double)(y1 - y0);
                    x = x0 + static_cast<int>((x1 - x0) * t);
                    y = ymax;
                } else if (outcodeOut & BOTTOM) { // point is below the clip rectangle
                    t = (ymin - y0) / (double)(y1 - y0);
                    x = x0 + static_cast<int>((x1 - x0) * t);
                    y = ymin;
                } else if (outcodeOut & RIGHT) {  // point is to the right of clip rectangle
                    t = (xmax - x0) / (double)(x1 - x0);
                    y = y0 + static_cast<int>((y1 - y0) * t);
                    x = xmax;
                } else if (outcodeOut & LEFT) {   // point is to the left of clip rectangle
                    t = (xmin - x0) / (double)(x1 - x0);
                    y = y0 + static_cast<int>((y1 - y0) * t);
                    x = xmin;
                }
                intersections += 1;

                if(intersections == 1)
                {
                    if(t1_out != nullptr)
                    {
                        *t1_out = t;
                    }
                    if(intersection_point1 != nullptr)
                    {
                        intersection_point1->x = x;
                        intersection_point1->y = y;
                    }
                } else {
                    if(t2_out != nullptr)
                    {
                        *t2_out = t;
                    }
                    if(intersection_point2 != nullptr)
                    {
                        intersection_point2->x = x;
                        intersection_point2->y = y;
                    }
                }

                // Now we move outside point to intersection point to clip
                // and get ready for next pass.
                if (outcodeOut == outcode0) {
                    x0 = x;
                    y0 = y;
                    outcode0 = out_code(x0, y0, xmin, xmax, ymin, ymax);
                } else {
                    x1 = x;
                    y1 = y;
                    outcode1 = out_code(x1, y1, xmin, xmax, ymin, ymax);
                }
            }
        }
        return intersections;
    }
    int seg_rect_intersection(const segment_i &seg,
                               const rectangle_i &rect,
                              point_i *intersection_point1,
                              point_i *intersection_point2,
                              double *t1_out,
                              double *t2_out)
    {
        return c_s_seg_rect_intersection(seg.start.x, seg.start.y,
                                         seg.end.x, seg.end.y,
                                         rect.left_bottom.x,
                                         rect.right_up().x,
                                         rect.left_bottom.y,
                                         rect.right_up().y,
                                         intersection_point1,
                                         intersection_point2,
                                         t1_out, t2_out);
    }
} // namespace geom2
