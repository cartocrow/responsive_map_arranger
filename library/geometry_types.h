#pragma once

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