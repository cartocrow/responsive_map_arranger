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
using Vec = Vector<Inexact>;
using Pt = Point<Inexact>;


class ChoroplethMap {
    struct MapComponent {
        std::vector<size_t> members;

        Pt position;
        Pt originalPosition;
        Pt cartogramPosition;
        Vec force{0, 0};
    };
    struct MapElement {

        optional<Region> region;
        optional<Rectangle<Inexact>> bb;

        Pt position;
        Pt originalPosition;
        Pt cartogramPosition;

        Vec force{0, 0};

        Color color{255, 255, 255};
    };

public:
    ChoroplethMap(const shared_ptr<RegularEdgeLabeling> &rel, RegionMap  regionMap) : m_REL(rel),
        m_map(std::move(regionMap)), rectangularDual(RectangularDual(rel)) {
    };

    void setFromRel();
    void runLayout(const size_t iterations);

    MapElement getMapElement(const size_t index) { return mapElements[index]; }

    // forces
    size_t forceIterationCount;
    double forceStepSize;
    double forceMaxMovement;
    double originalPosForce;
    double cartogramPosForce;
    double RELForce;
    double overlapForce;
    double boundaryForce ;

private:
    void setRegions();
    void normalizeMap(const double areaFraction);
    void saveOriginalPositions();
    void setCartogramPositions();
    void setInitialPositions();
    void initializeComponentPositions();
    void buildComponents();

    // Improving map layout
    void clearForces();
    void computeOriginalPositionForces();
    void computeCartogramPositionForces();
    void computeRELForces();
    void computeOverlapForces();
    void computeBoundaryForces();
    bool applyForces();

    void applyHorizontalConstraint(size_t left, size_t right, size_t leftComponent, size_t rightComponent);
    void applyVerticalConstraint(size_t bottom, size_t top, size_t bottomComponent, size_t topComponent);

    void translateRegion(MapElement& element, const Vec& delta);
    void translateComponent(size_t componentIndex, const Vec& delta);
    void translateComponent(MapComponent component, const Vec& delta);
    void transformRegion(MapElement& element, const CGAL::Aff_transformation_2<Inexact>& transformation);

    optional<Rect> mapBoundingBox() const;

private:
    shared_ptr<RegularEdgeLabeling> m_REL;
    RegionMap m_map;

    Rect container;
    RectangularDual rectangularDual;

    std::vector<MapElement> mapElements;
    vector<MapComponent> mapComponents;

    vector<int> componentOfElement;

    double forceThreshold = 1e-4;

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

