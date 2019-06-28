#include "point2d.h"

Point2D::Point2D()
{
    x = y = 0.0;
}

Point2D::Point2D(double _x, double _y)
{
    x = _x; y = _y;
}

Point2D Point2D::operator +(const Point2D &point) const
{
    return Point2D(x + point.x, y + point.y);
}

Point2D Point2D::operator -(const Point2D &point) const
{
    return Point2D(x - point.x, y - point.y);
}

Point2D Point2D::operator *(double v) const
{
    return Point2D(x * v, y * v);
}

void Point2D::operator +=(const Point2D &point)
{
    x += point.x; y += point.y;
}

void Point2D::operator -=(const Point2D &point)
{
    x -= point.x; y -= point.y;
}

void Point2D::normalize()
{
    double l = sqrt(x * x + y * y);
    x /= l;
    y /= l;
}
