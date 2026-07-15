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

#pragma once

#include <utility>

#include "regular_edge_labeling.h"
#include "rectangular_dual.h"
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>

using namespace cartocrow;
using namespace std;
using Rect = Rectangle<Inexact>;

class ChoroplethMap {
    struct MapElement {
        using Pt = Point<cartocrow::Inexact>;

        optional<Region> region;
        optional<Rect> bb;
        Pt position;
        Color color{255, 255, 255};
    };

public:
    ChoroplethMap(const shared_ptr<RegularEdgeLabeling> &rel, RegionMap  regionMap) : m_REL(rel),
        m_map(std::move(regionMap)), rectangularDual(RectangularDual(rel)) {
    };

    void setFromRel();
    void setRegions();
    void scaleRegionsToContainer();
    void setInitialPositions();

private:
    shared_ptr<RegularEdgeLabeling> m_REL;
    RegionMap m_map;

    Rect container;
    RectangularDual rectangularDual;

    std::vector<MapElement> mapElements;

    friend class ChoroplethPainting;
};

class ChoroplethPainting : public renderer::GeometryPainting{
public:
    using Renderer = renderer::GeometryRenderer;

    explicit ChoroplethPainting(std::shared_ptr<ChoroplethMap> map,
                               std::shared_ptr<RegularEdgeLabeling> relmap = nullptr) : m_map(std::move(map)), m_REL(std::move(relmap)) {};
    virtual ~ChoroplethPainting() = default;

    void paint(Renderer& renderer) const override;

private:
    std::shared_ptr<ChoroplethMap> m_map;
    std::shared_ptr<RegularEdgeLabeling> m_REL = nullptr;
};

