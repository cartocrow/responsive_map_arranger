/*
Copyright (C) 2026  TU Eindhoven

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <type_traits>

#include "choropleth_map.h"
#include "cartocrow/core/transform_helpers.h"

using Transformation = CGAL::Aff_transformation_2<Inexact>;

void ChoroplethMap::setFromRel() {
    assert(rel.hasBoundingBox());
    auto [left, right, bottom, top] = m_REL->getBoundingBox().value();
    container = Rectangle<Inexact>(left, bottom, right, top);


    const auto &vertices = m_REL->getVertices();
    const int V = static_cast<int>(vertices.size());

    // Resize rects to number of vertices
    mapElements.clear();
    mapElements.resize(static_cast<size_t>(V));

    for (int v = 4; v < V; ++v) {
        MapElement element;
        element.color = vertices[v].color;
        mapElements[v] = element;
    }

    setRegions();
    scaleRegionsToContainer();
    setInitialPositions();
}

void ChoroplethMap::setRegions() {

    auto& vertices = m_REL->getVertices();

    std::unordered_map<string, size_t> indexByLabel;
    indexByLabel.reserve(vertices.size());

    for (size_t i = 0; i < vertices.size(); ++i) {
        indexByLabel.emplace(vertices[i].label, i);
    }

    for (const auto &[name, region] : m_map) {
        auto found = indexByLabel.find(name);

        if (found == indexByLabel.end()) {
            std::cout << "[WARNING]: region '" << region.name << "' not found in vertices\n";
            continue;
        }

        const size_t index = found->second;
        auto& mapElement = mapElements[index];

        mapElement.color = region.color;
        mapElement.bb = boundingBox(region.shape);
        mapElement.region = region;
    }

}

void ChoroplethMap::scaleRegionsToContainer() {
    double totalRegionBBArea = 0;

    for (const auto& element : mapElements) {
        if (!element.region || !element.bb) continue;

        totalRegionBBArea += element.bb->area();
    }

    if (totalRegionBBArea <= 0.0) return;

    const double scaleFactor = sqrt(container.area() / totalRegionBBArea);

    const Transformation scale{CGAL::SCALING, scaleFactor};

    for (auto& element : mapElements) {
        if (!element.region || !element.bb) continue;

        const Rect oldBox = *element.bb;

        const Point<Inexact> center{
            (oldBox.xmin() + oldBox.xmax()) * 0.5,
            (oldBox.ymin() + oldBox.ymax()) * 0.5
        };

        const Transformation moveToOrigin{CGAL::TRANSLATION, CGAL::ORIGIN - center };
        const Transformation moveBack{CGAL::TRANSLATION, center - CGAL::ORIGIN };

        const auto transformation = moveBack * scale * moveToOrigin;

        auto inexactShape = approximate(element.region->shape);
        inexactShape = transform(transformation, inexactShape);
        element.region->shape = pretendExact(inexactShape);

        element.bb = boundingBox(element.region->shape);
    }
}

void ChoroplethMap::setInitialPositions() {
    rectangularDual.setFromREL();
    auto& rects = rectangularDual.rectangles();

    for (size_t i = 0; i < mapElements.size(); ++i) {
        auto& element = mapElements[i];

        if (!element.region || !element.bb) {
            continue;
        }

        element.position = rects[i].center();


        const auto target = rects[i].center();
        const auto& regionBB = *element.bb;

        const double sourceX = (regionBB.xmin() + regionBB.xmax()) * 0.5;
        const double sourceY = (regionBB.ymin() + regionBB.ymax()) * 0.5;

        const Vector<Inexact> offset{
            target.x() - sourceX,
            target.y() - sourceY
        };

        const CGAL::Aff_transformation_2<Inexact> translation{
            CGAL::TRANSLATION,
            offset
        };

        auto shape = approximate(element.region->shape);
        shape = transform(translation, shape);
        element.region->shape = pretendExact(shape);
        element.bb = CGAL::Bbox_2{
            regionBB.xmin() + offset.x(),
            regionBB.ymin() + offset.y(),
            regionBB.xmax() + offset.x(),
            regionBB.ymax() + offset.y()
        };
    }
}

void ChoroplethPainting::paint(Renderer& renderer) const {

    if (!m_map) {
        return;
    }

    renderer.setMode(Renderer::fill | Renderer::stroke);

    auto& relVertices = m_REL->getVertices();
    auto& mapElements = m_map->mapElements;
    for (size_t i = 4; i < relVertices.size(); i++) {
        if (!relVertices[i].isLandRegion) continue;
        //DemersPosition dp = m_cartogram->locations[i];
        //cout << mapElements[i].color.r << " " << mapElements[i].color.g << " " << mapElements[i].color.b << endl;
        renderer.setFill(mapElements[i].color);
        renderer.draw(mapElements[i].region->shape);

        // if (m_drawLabels) {
        //     renderer.setFill({ 0,0,0 });
        //     renderer.drawText(dp.center, dp.label);
        // }
    }

    // draw boundingbox
    renderer.setStroke({ 102,102,102 }, 2);
    renderer.setMode(Renderer::stroke);
    renderer.draw(m_map->container);
};
