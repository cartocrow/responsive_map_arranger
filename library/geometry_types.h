#pragma once

#include "cartocrow/core/rectangle_helpers.h"

struct BoundingBox
{
    double left   = 0;
    double right  = 0;
    double bottom = 0;
    double top    = 0;

    double width()  const { return right - left; }
    double height() const { return top - bottom; }
    double area() const { return (right - left) * (top - bottom); }
};

template <class K>
CGAL::Bbox_2 boundingBox(const cartocrow::PolygonSet<K>& shape)
{
    std::vector<cartocrow::PolygonWithHoles<K>> polygons;
    shape.polygons_with_holes(std::back_inserter(polygons));

    if (polygons.empty()) {
        return {};
    }

    CGAL::Bbox_2 result = polygons.front().outer_boundary().bbox();

    for (std::size_t i = 1; i < polygons.size(); ++i) {
        result = result + polygons[i].outer_boundary().bbox();
    }

    return result;
}

inline cartocrow::Rectangle<cartocrow::Inexact> shrinkRectangle(const cartocrow::Rectangle<cartocrow::Inexact>& r, double factor)
{
    const auto c = cartocrow::centroid(r);

    const double halfWidth  = cartocrow::width(r) * factor * 0.5;
    const double halfHeight = cartocrow::height(r) * factor * 0.5;

    return {
        cartocrow::Point<cartocrow::Inexact>{
            c.x() - halfWidth,
            c.y() - halfHeight
        },
        cartocrow::Point<cartocrow::Inexact>{
            c.x() + halfWidth,
            c.y() + halfHeight
        }
    };
}