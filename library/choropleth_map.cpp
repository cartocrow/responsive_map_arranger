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

void ChoroplethMap::setFromRel() {
    assert(rel.hasBoundingBox());
    bb = m_REL->getBoundingBox().value();
    box = Rectangle<Inexact>(bb.left, bb.bottom, bb.right, bb.top);


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
        auto& vertex = vertices[index];
        auto& mapElement = mapElements[index];

        std::vector<PolygonWithHoles<Exact>> polygons;
        region.shape.polygons_with_holes(std::back_inserter(polygons));

        std::vector<std::pair<PolygonWithHoles<Exact>, Number<Exact>>> parts;

        for (const auto &p : polygons) {
            parts.push_back(pair(p, area(p)));
        }

        std::sort(parts.begin(), parts.end(), [](const auto &p1, const auto &p2) {
            return p1.second > p2.second;
        });

        auto regionBB = parts.front().first.bbox();

        mapElement.color = region.color;
        mapElement.bb = regionBB;
        mapElement.region = region;
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

        using K = cartocrow::Exact;

        const auto target = rects[i].center();
        const auto& regionBB = *element.bb;

        const double sourceX = (regionBB.xmin() + regionBB.xmax()) * 0.5;
        const double sourceY = (regionBB.ymin() + regionBB.ymax()) * 0.5;

        const cartocrow::Vector<K> offset{
            K::FT(target.x() - sourceX),
            K::FT(target.y() - sourceY)
        };

        const CGAL::Aff_transformation_2<K> translation{
            CGAL::TRANSLATION,
            offset
        };


        element.region->shape = cartocrow::transform(translation, element.region->shape);
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
    renderer.draw(m_map->box);
};
