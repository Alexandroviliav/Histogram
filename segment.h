#ifndef SEGMENT_H
#define SEGMENT_H

#include "point2d.h"

class Segment
{
public:
    Segment();
    Point2D points[4];

    void calc(double, Point2D &);
};

#endif // SEGMENT_H
