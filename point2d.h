#ifndef POINT2D_H
#define POINT2D_H

#include <vector>
#include <cmath>

class Point2D
{
public:
    Point2D();
    Point2D(double, double);

    double x, y;

    Point2D operator +(const Point2D &) const;
    Point2D operator -(const Point2D &) const;
    Point2D operator *(double) const;
    void operator    +=(const Point2D &);
    void operator    -=(const Point2D &);

    void normalize();
};

#endif // POINT2D_H
