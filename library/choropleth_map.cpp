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
#include "cartocrow/core/polygon_helpers.h"

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
    normalizeMap(0.7);
    saveOriginalPositions();
    setCartogramPositions();
    buildComponents();
    initializeComponentPositions();

    runLayout(forceIterationCount);
}

void ChoroplethMap::runLayout(const size_t iterations) {
    for (size_t i = 0; i < iterations; ++i) {
        clearForces();

        computeOriginalPositionForces();
        computeCartogramPositionForces();
        computeRELForces();
        computeOverlapForces();
        computeBoundaryForces();

        if (applyForces()) break;
    }
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

void ChoroplethMap::normalizeMap(const double areaFraction) {
    if (areaFraction <= 0.0 || areaFraction > 1.0) return;

    const Rect mapBB = *mapBoundingBox();

    double totalRegionArea = 0.0;
    double largestRegionWidth = 0.0;
    double largestRegionHeight = 0.0;

    for (const auto& element : mapElements) {
        if (!element.region || !element.bb) continue;
        totalRegionArea += CGAL::to_double(element.bb->area());
        largestRegionWidth = max(largestRegionWidth, width(*element.bb));
        largestRegionHeight = max(largestRegionHeight, height(*element.bb));
    }

    if (totalRegionArea <= 0.0) return;

    const double containerArea = CGAL::to_double(container.area());
    const double containerWidth = CGAL::to_double(cartocrow::width(container));
    const double containerHeight = CGAL::to_double(cartocrow::height(container));


    const double areaScale = std::sqrt(containerArea / totalRegionArea) * areaFraction;
    const double widthScale = containerWidth / largestRegionWidth;
    const double heightScale = containerHeight / largestRegionHeight;

    const double scaleFactor = std::min({areaScale, widthScale, heightScale});
    const Transformation scale{CGAL::SCALING, scaleFactor};

    const Pt mapCenter = centroid(mapBB);
    const Pt containerCenter = centroid(container);

    const Transformation moveToOrigin{ CGAL::TRANSLATION, CGAL::ORIGIN - mapCenter };
    const Transformation moveToContainer{ CGAL::TRANSLATION, containerCenter - CGAL::ORIGIN };
    const Transformation transformation = moveToContainer * scale * moveToOrigin;

    for (auto& element : mapElements) {
        if (!element.region || !element.bb) continue;
        transformRegion(element, transformation);
    }
}

void ChoroplethMap::saveOriginalPositions() {
    for (auto& element : mapElements) {
        if (!element.region || !element.bb) continue;

        element.position = centroid(approximate(element.region->shape));
        element.originalPosition = element.position;
    }
}

void ChoroplethMap::setCartogramPositions() {
    rectangularDual.setFromREL();
    const auto& rects = rectangularDual.rectangles();

    const std::size_t count =
        std::min(mapElements.size(), rects.size());

    for (std::size_t i = 0; i < count; ++i) {
        auto& element = mapElements[i];
        if (!element.region || !element.bb) continue;

        element.cartogramPosition = rects[i].center();
    }
}

void ChoroplethMap::setInitialPositions() {
    rectangularDual.setFromREL();
    const auto& rects = rectangularDual.rectangles();

    const std::size_t count =
        std::min(mapElements.size(), rects.size());

    for (std::size_t i = 0; i < count; ++i) {
        auto& element = mapElements[i];

        if (!element.region || !element.bb) continue;

        //const auto target = rects[i].center();

        element.cartogramPosition = rects[i].center();

        //const auto delta = target - element.position;
        //translateRegion(element, delta);
    }
}

void ChoroplethMap::initializeComponentPositions() {
    for (MapComponent& component : mapComponents) {
        if (component.members.empty()) continue;

        double currentX = 0.0;
        double currentY = 0.0;
        double originalX = 0.0;
        double originalY = 0.0;
        double cartogramX = 0.0;
        double cartogramY = 0.0;

        for (const std::size_t elementIndex : component.members) {

            const MapElement& element = mapElements[elementIndex];

            currentX += element.position.x();
            currentY += element.position.y();
            originalX += element.originalPosition.x();
            originalY += element.originalPosition.y();
            cartogramX += element.cartogramPosition.x();
            cartogramY += element.cartogramPosition.y();
             }

        const double count = static_cast<double>(component.members.size());
        component.position = Pt( currentX / count, currentY / count);
        component.originalPosition = Pt( originalX / count, originalY / count);
        component.cartogramPosition = Pt( cartogramX / count, cartogramY / count);

        component.force = Vec(0.0, 0.0);

        translateComponent(component, component.cartogramPosition - component.position);
    }
}

void ChoroplethMap::buildComponents() {
    const vector<int> componentOfVertex = m_REL->componentOfVertex();

    mapComponents.clear();
    componentOfElement.assign(componentOfVertex.size(), -1);

    int componentCount = 0;

    for (const int componentId : componentOfVertex) {
        if (componentId >= 0) {
            componentCount = max(componentCount, componentId + 1);
        }
    }

    mapComponents.resize(componentCount);
    const size_t elementCount = min(mapElements.size(), componentOfVertex.size());

    for (size_t i = 0; i < elementCount; ++i) {
        const int componentId = componentOfVertex[i];

        if (componentId < 0) continue;

        componentOfElement[i] = componentId;

        mapComponents[componentId].members.push_back(i);
    }

    initializeComponentPositions();
}

void ChoroplethMap::clearForces() {
    for (auto& element : mapElements) {
        element.force = {0,0};
    }

    for (auto& component : mapComponents) {
        component.force = {0,0};
    }
}

void ChoroplethMap::computeOriginalPositionForces() {
    if (originalPosForce < forceThreshold) return;

    // for (auto& element : mapElements) {
    //     if (!element.region || !element.bb) continue;
    //
    //     const Vec displacement = element.originalPosition - element.position;
    //
    //     element.force = element.force + Vec{
    //         originalPosForce * displacement.x(),
    //         originalPosForce * displacement.y()
    //     };
    // }

    for (auto& component : mapComponents) {
        const Vec displacement = component.originalPosition - component.position;
        component.force = component.force + originalPosForce * displacement;
    }
}

void ChoroplethMap::computeCartogramPositionForces() {
    if (cartogramPosForce < forceThreshold) return;

    // for (auto& element : mapElements) {
    //     if (!element.region || !element.bb) continue;
    //
    //     const Vec displacement = element.cartogramPosition - element.position;
    //
    //     element.force = element.force + Vec{
    //         cartogramPosForce * displacement.x(),
    //         cartogramPosForce * displacement.y()
    //     };
    // }

    for (auto& component : mapComponents) {
        const Vec displacement = component.cartogramPosition - component.position;
        component.force = component.force + cartogramPosForce * displacement;
    }
}

void ChoroplethMap::computeRELForces() {
    if (RELForce < forceThreshold) return;

    const vector<HalfEdge>& halfEdges = m_REL->getHalfEdges();
    const vector<Vertex>& vertices = m_REL->getVertices();

    for (size_t edgeId = 0; edgeId < halfEdges.size(); ++edgeId) {
         const auto& edge = halfEdges[edgeId];
         if (edge.isDeleted || edge.color == BLACK || !edge.outgoing) continue;

         const int source = edge.vertex;
         const int target = m_REL->neighborOfHalfEdge(static_cast<int>(edgeId));
         const Vertex sourceVertex = vertices[source];
         const Vertex targetVertex = vertices[target];

         if (source < 4 || target < 4 ) continue; // skip outer vertices
         if (!sourceVertex.isLandRegion || !targetVertex.isLandRegion) continue;

         const auto sourceIndex = static_cast<size_t>(source);
         const auto targetIndex = static_cast<size_t>(target);

        const int sourceComponent = componentOfElement[sourceIndex];
        const int targetComponent = componentOfElement[targetIndex];

        // do not apply force between regions of same element.
        if (sourceComponent == targetComponent ||
            sourceComponent < 0 || targetComponent < 0 ||
            sourceComponent >= mapComponents.size() || targetComponent >= mapComponents.size()) continue;

        if (edge.color == BLUE) { //horizontal adjacency
            applyHorizontalConstraint(sourceIndex, targetIndex, sourceComponent, targetComponent);
        } else if (edge.color == RED) { //vertical adjacency
            applyVerticalConstraint(sourceIndex, targetIndex, sourceComponent, targetComponent);
        }
    }
}

void ChoroplethMap::computeOverlapForces() {
    if (overlapForce < forceThreshold) return;

    for (size_t i = 0; i < mapElements.size(); ++i) {
        auto& a = mapElements[i];

        if (!a.region || !a.bb) continue;

        for (size_t j = i + 1; j < mapElements.size(); ++j) {
            auto& b = mapElements[j];
            if (!b.region || !b.bb) continue;

            const double overlapX = min(a.bb->xmax(), b.bb->xmax()) - max(a.bb->xmin(), b.bb->xmin());
            const double overlapY = min(a.bb->ymax(), b.bb->ymax()) - max(a.bb->ymin(), b.bb->ymin());

            if (overlapX <= 0 || overlapY <= 0) continue;

            if (overlapX < overlapY && overlapX > 0) {
                const double dir = a.position.x() < b.position.x() ? -1 : 1;
                const double magnitude = overlapForce * overlapX;

                const int aComponent = componentOfElement[i];
                const int bComponent = componentOfElement[j];

                mapComponents[aComponent].force += Vec{dir * magnitude, 0 };
                mapComponents[bComponent].force += Vec{-dir * magnitude, 0 };
            } else {
                const double dir = a.position.y() < b.position.y() ? -1 : 1;
                const double magnitude = overlapForce * overlapY;

                const int aComponent = componentOfElement[i];
                const int bComponent = componentOfElement[j];

                mapComponents[aComponent].force += Vec{0, dir * magnitude};
                mapComponents[bComponent].force += Vec{0, -dir * magnitude};
            }

        }
    }
}

void ChoroplethMap::computeBoundaryForces() {
    if (boundaryForce < forceThreshold) return;

    for (size_t i = 0; i < mapElements.size(); ++i) {
        auto& element = mapElements[i];
        if (!element.region || !element.bb) continue;

        double forceX = 0.0;
        double forceY = 0.0;

        // left side
        if (element.bb->xmin() < container.xmin()) {
            forceX += boundaryForce * (container.xmin()-element.bb->xmin());
        }
        // right side
        if (element.bb->xmax() > container.xmax()) {
            forceX -= boundaryForce * (element.bb->xmax()-container.xmax());
        }
        // bottom side
        if (element.bb->ymin() < container.ymin()) {
            forceY += boundaryForce * (container.ymin()-element.bb->ymin());
        }
        // top side
        if (element.bb->ymax() > container.ymax()) {
            forceY -= boundaryForce * (element.bb->ymax()-container.ymax());
        }
        auto& component = mapComponents[componentOfElement[i]];
        component.force += Vec{forceX, forceY};
    }
}

bool ChoroplethMap::applyForces() {
    double largestMovement = 0.0;
    for (auto& component : mapComponents) {
        //if (!element.region || !element.bb) continue;

        double dx = forceStepSize * component.force.x();
        double dy = forceStepSize * component.force.y();

        const double length = sqrt(dx * dx + dy * dy);

        if (length > forceMaxMovement) {
            const double factor = forceMaxMovement / length;
            dx *= factor;
            dy *= factor;
        }
        largestMovement = max(largestMovement, hypot(dx, dy));

        //if (abs(dx) < 1e-9 && abs(dy) < 1e-9) continue;

        translateComponent(component, Vec{dx, dy});
    }

    return largestMovement < forceThreshold;
}

void ChoroplethMap::applyHorizontalConstraint(size_t left, size_t right, size_t leftComponent, size_t rightComponent) {
    auto& a = mapElements[left];
    auto& b = mapElements[right];

    if (!a.bb || !b.bb) return;

    const double allowedOverlap = 0.05 * min(width(*a.bb), width(*b.bb));

    // positive: bb gap | zero: bb touch | negative: bb overlap
    const double gap = b.bb->xmin() - a.bb->xmax() + allowedOverlap;

    const double contactForce = RELForce * gap;

    mapComponents[leftComponent].force += Vec{ contactForce, 0.0 };
    mapComponents[rightComponent].force += Vec{ -contactForce, 0.0 };

    // const double verticalDifference = b.position.y() - a.position.y();
    // const double alignmentForce = alignmentStrength * verticalDifference;
    //
    // a.force = a.force + Vec{ 0.0, alignmentForce };
    // b.force = b.force + Vec{ 0.0, -alignmentForce };
}

void ChoroplethMap::applyVerticalConstraint(size_t bottom, size_t top, size_t bottomComponent, size_t topComponent) {
    auto& a = mapElements[bottom];
    auto& b = mapElements[top];

    if (!a.bb || !b.bb) return;

    const double allowedOverlap = 0.05 * min(height(*a.bb), height(*b.bb));
    const double gap = b.bb->ymin() - a.bb->ymax() + allowedOverlap;

    const double contactForce = RELForce * gap;

    mapComponents[bottomComponent].force += Vec{ 0.0, contactForce };
    mapComponents[topComponent].force += Vec{ 0.0, -contactForce };

    // const double horizontalDifference = b.position.x() - a.position.x();
    // const double alignmentForce = alignmentStrength * horizontalDifference;
    //
    // a.force = a.force + Vec{ alignmentForce, 0.0 };
    // b.force = b.force + Vec{ -alignmentForce, 0.0 };
}

void ChoroplethMap::translateRegion(MapElement& element, const Vec& translation) {
    if (!element.region || !element.bb) {
        return;
    }

    const CGAL::Aff_transformation_2<Inexact> transformation{
        CGAL::TRANSLATION,
        translation
    };

    auto inexactShape =
        approximate(element.region->shape);

    inexactShape =
        cartocrow::transform(transformation, inexactShape);

    element.region->shape =
        pretendExact(inexactShape);

    const Rect oldBox = *element.bb;

    element.bb = Rect{
        oldBox.vertex(0) + translation,
        oldBox.vertex(2) + translation
    };

    element.position = element.position + translation;
}

void ChoroplethMap::translateComponent(const size_t componentIndex, const Vec &delta) {
    const MapComponent& component = mapComponents.at(componentIndex);
    translateComponent(component, delta);
}

void ChoroplethMap::translateComponent(MapComponent component, const Vec &delta) {
    for (const size_t i : component.members) {
        translateRegion(mapElements[i], delta);
    }
    component.position += delta;
}

void ChoroplethMap::transformRegion(MapElement &element, const CGAL::Aff_transformation_2<Inexact> &transformation) {
    if (!element.region || !element.bb) return;

    auto inexactShape = approximate(element.region->shape);

    inexactShape = transform(transformation, inexactShape);
    element.region->shape = pretendExact(inexactShape);

    element.bb = boundingBox(element.region->shape);

    element.position = centroid(approximate(element.region->shape));
}

optional<Rect> ChoroplethMap::mapBoundingBox() const {
    std::optional<Rect> result;

    for (const auto& element : mapElements) {
        if (!element.region || !element.bb) {
            continue;
        }

        if (!result) {
            result = *element.bb;
            continue;
        }

        result = Rect{
            Pt{
                std::min(result->xmin(), element.bb->xmin()),
                std::min(result->ymin(), element.bb->ymin())
            },
            Pt{
                std::max(result->xmax(), element.bb->xmax()),
                std::max(result->ymax(), element.bb->ymax())
            }
        };
    }

    return result;
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
