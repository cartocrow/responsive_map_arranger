#pragma once

struct BoundingBox
{
    double left   = 0;
    double right  = 0;
    double bottom = 0;
    double top    = 0;

    double width()  const { return right - left; }
    double height() const { return top - bottom; }
};