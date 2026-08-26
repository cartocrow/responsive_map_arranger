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
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

#include "choropleth_map.h"
#include "cartocrow/core/transform_helpers.h"
#include "cartocrow/core/polygon_helpers.h"

using Transformation = CGAL::Aff_transformation_2<Inexact>;

namespace {

Color choroplethRampColor(const int index) {
    static const std::array<Color, 5> kRamp = {
        Color{0xef, 0xf3, 0xff},
        Color{0xbd, 0xd7, 0xe7},
        Color{0x6b, 0xae, 0xd6},
        Color{0x31, 0x82, 0xbd},
        Color{0x08, 0x51, 0x9c},
    };

    return kRamp[std::clamp(index, 0, static_cast<int>(kRamp.size()) - 1)];
}

}

void ChoroplethMap::setFromRel() {
    assert(m_REL->hasBoundingBox());
    auto [left, right, bottom, top] = m_REL->getBoundingBox().value();
    container = Rectangle<Inexact>(left, bottom, right, top);


    const auto &vertices = m_REL->getVertices();
    const int V = static_cast<int>(vertices.size());

    // Resize rects to number of vertices
    mapElements.clear();
    mapElements.resize(static_cast<size_t>(V));

    for (int v = 4; v < V; ++v) {
        MapElement element;
        element.baseColor = vertices[v].color;
        element.fillColor = vertices[v].color;
        mapElements[v] = element;
    }

    setRegions();
    updateFillColors();
    normalizeMap(0.7);
    saveOriginalPositions();
    setCartogramPositions();
    buildComponents();
    initializeComponentMetadata();
    placeComponentsFromCartogram();

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

        mapElement.baseColor = region.color;
        mapElement.fillColor = region.color;
        mapElement.bb = boundingBox(region.shape);
        mapElement.region = region;
    }

}

void ChoroplethMap::setUseValueColors(const bool useValueColors) {
    m_useValueColors = useValueColors;
    updateFillColors();
}

std::unordered_map<std::string, Pt> ChoroplethMap::regionCentroids() const {
    std::unordered_map<std::string, Pt> centroids;

    for (const auto &element : mapElements) {
        if (!element.region) continue;
        if (element.region->name.empty()) continue;
        centroids[element.region->name] = centroid(approximate(element.region->shape));
    }

    return centroids;
}

void ChoroplethMap::updateFillColors() {
    if (!m_useValueColors) {
        for (auto& element : mapElements) {
            element.fillColor = element.baseColor;
        }
        return;
    }

    const auto& vertices = m_REL->getVertices();
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();

    for (size_t i = 4; i < mapElements.size() && i < vertices.size(); ++i) {
        if (!mapElements[i].region || !vertices[i].isLandRegion) continue;

        const double value = std::isfinite(vertices[i].oldWeight) ? vertices[i].oldWeight : vertices[i].weight;
        if (!std::isfinite(value)) continue;

        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }

    if (!std::isfinite(minValue) || !std::isfinite(maxValue)) {
        for (auto& element : mapElements) {
            element.fillColor = element.baseColor;
        }
        return;
    }

    const double range = maxValue - minValue;
    for (size_t i = 4; i < mapElements.size() && i < vertices.size(); ++i) {
        auto& element = mapElements[i];
        if (!element.region || !vertices[i].isLandRegion) continue;

        const double value = std::isfinite(vertices[i].oldWeight) ? vertices[i].oldWeight : vertices[i].weight;
        if (!std::isfinite(value) || range <= 0.0) {
            element.fillColor = choroplethRampColor(0);
            continue;
        }

        const double normalized = std::clamp((value - minValue) / range, 0.0, 1.0);
        const int bucket = std::min(4, static_cast<int>(normalized * 5.0));
        element.fillColor = choroplethRampColor(bucket);
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

    for (std::size_t i = 0; i < mapElements.size(); ++i) {
        auto& element = mapElements[i];
        if (!element.region || !element.bb) continue;

        element.cartogramPosition = rects[i].center();
    }
}

void ChoroplethMap::initializeComponentMetadata() {
    for (MapComponent& component : mapComponents) {
        if (component.members.empty()) continue;

        double currentX = 0.0;
        double currentY = 0.0;
        double originalX = 0.0;
        double originalY = 0.0;
        double cartogramX = 0.0;
        double cartogramY = 0.0;

        for (const size_t elementIndex : component.members) {
            const MapElement& element = mapElements[elementIndex];
            currentX += element.position.x();
            currentY += element.position.y();
            originalX += element.originalPosition.x();
            originalY += element.originalPosition.y();
            cartogramX += element.cartogramPosition.x();
            cartogramY += element.cartogramPosition.y();
        }

        const double count = static_cast<double>(component.members.size());
        component.position = Pt(currentX / count, currentY / count);
        component.originalPosition = Pt(originalX / count, originalY / count);
        component.cartogramPosition = Pt(cartogramX / count, cartogramY / count);
        component.force = Vec(0.0, 0.0);
        component.velocity = Vec(0.0, 0.0);
    }
}

void ChoroplethMap::buildComponents() {
    const vector<int> componentOfVertex = m_REL->componentOfVertex();

    mapComponents.clear();
    componentOfElement.assign(componentOfVertex.size(), -1);

    const size_t elementCount = min(mapElements.size(), componentOfVertex.size());
    unordered_map<int, int> componentRemap;

    for (size_t i = 0; i < elementCount; ++i) {
        if (!mapElements[i].region || !mapElements[i].bb) continue;

        const int componentId = componentOfVertex[i];
        if (componentId < 0) continue;

        auto [it, inserted] = componentRemap.emplace(componentId, static_cast<int>(mapComponents.size()));
        if (inserted) {
            mapComponents.emplace_back();
        }

        componentOfElement[i] = it->second;
        mapComponents[it->second].members.push_back(i);
    }
}

void ChoroplethMap::placeComponentsFromCartogram() {
    const size_t componentCount = mapComponents.size();
    if (componentCount == 0) return;

    for (size_t i = 0; i < componentCount; ++i) {
        auto& component = mapComponents[i];
        Vec averageDisplacement{0.0, 0.0};

        for (const size_t elementIndex : component.members) {
            averageDisplacement = averageDisplacement
                + (mapElements[elementIndex].cartogramPosition - mapElements[elementIndex].originalPosition);
        }

        averageDisplacement = averageDisplacement / static_cast<double>(component.members.size());
        translateComponent(component, averageDisplacement);
    }
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

    for (auto& component : mapComponents) {
        const Vec displacement = component.originalPosition - component.position;
        component.force = component.force + originalPosForce * displacement;
    }
}

void ChoroplethMap::computeCartogramPositionForces() {
    if (cartogramPosForce < forceThreshold) return;

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

    for (size_t i = 0; i < mapComponents.size(); ++i) {
        const auto boundsA = componentBoundingBox(mapComponents[i]);
        if (!boundsA) continue;

        for (size_t j = i + 1; j < mapComponents.size(); ++j) {
            const auto boundsB = componentBoundingBox(mapComponents[j]);
            if (!boundsB) continue;

            const double componentOverlapX = min(boundsA->xmax(), boundsB->xmax()) - max(boundsA->xmin(), boundsB->xmin());
            const double componentOverlapY = min(boundsA->ymax(), boundsB->ymax()) - max(boundsA->ymin(), boundsB->ymin());
            if (componentOverlapX <= 0.0 || componentOverlapY <= 0.0) continue;

            double regionOverlapX = 0.0;
            double regionOverlapY = 0.0;

            for (const size_t memberA : mapComponents[i].members) {
                const auto& elementA = mapElements[memberA];
                if (!elementA.bb) continue;

                for (const size_t memberB : mapComponents[j].members) {
                    const auto& elementB = mapElements[memberB];
                    if (!elementB.bb) continue;

                    const double overlapX = min(elementA.bb->xmax(), elementB.bb->xmax())
                                          - max(elementA.bb->xmin(), elementB.bb->xmin());
                    const double overlapY = min(elementA.bb->ymax(), elementB.bb->ymax())
                                          - max(elementA.bb->ymin(), elementB.bb->ymin());
                    if (overlapX <= 0.0 || overlapY <= 0.0) continue;

                    regionOverlapX += overlapX;
                    regionOverlapY += overlapY;
                }
            }

            if (regionOverlapX <= 0.0 || regionOverlapY <= 0.0) continue;

            if (regionOverlapX <= regionOverlapY) {
                const double dir = mapComponents[i].position.x() <= mapComponents[j].position.x() ? -1.0 : 1.0;
                const double magnitude = overlapForce * regionOverlapX;
                mapComponents[i].force += Vec{dir * magnitude, 0.0};
                mapComponents[j].force += Vec{-dir * magnitude, 0.0};
            } else {
                const double dir = mapComponents[i].position.y() <= mapComponents[j].position.y() ? -1.0 : 1.0;
                const double magnitude = overlapForce * regionOverlapY;
                mapComponents[i].force += Vec{0.0, dir * magnitude};
                mapComponents[j].force += Vec{0.0, -dir * magnitude};
            }
        }
    }
}

void ChoroplethMap::computeBoundaryForces() {
    if (boundaryForce < forceThreshold) return;

    for (auto& component : mapComponents) {
        const auto bounds = componentBoundingBox(component);
        if (!bounds) continue;

        double forceX = 0.0;
        double forceY = 0.0;

        if (bounds->xmin() < container.xmin()) {
            forceX += boundaryForce * (container.xmin() - bounds->xmin());
        }
        if (bounds->xmax() > container.xmax()) {
            forceX -= boundaryForce * (bounds->xmax() - container.xmax());
        }
        if (bounds->ymin() < container.ymin()) {
            forceY += boundaryForce * (container.ymin() - bounds->ymin());
        }
        if (bounds->ymax() > container.ymax()) {
            forceY -= boundaryForce * (bounds->ymax() - container.ymax());
        }
        component.force += Vec{forceX, forceY};
    }
}

bool ChoroplethMap::applyForces() {
    double largestMovement = 0.0;
    constexpr double damping = 0.6;
    constexpr double minVelocity = 1e-5;

    for (auto& component : mapComponents) {
        double dx = damping * component.velocity.x() + forceStepSize * component.force.x();
        double dy = damping * component.velocity.y() + forceStepSize * component.force.y();

        const double length = sqrt(dx * dx + dy * dy);

        if (length > forceMaxMovement) {
            const double factor = forceMaxMovement / length;
            dx *= factor;
            dy *= factor;
        }
        const double movement = hypot(dx, dy);
        largestMovement = max(largestMovement, movement);

        component.velocity = Vec{dx, dy};
        if (movement < minVelocity) {
            component.velocity = Vec{0.0, 0.0};
        }
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
    if (!element.region || !element.bb) return;

    const CGAL::Aff_transformation_2<Inexact> transformation{
        CGAL::TRANSLATION,
        translation
    };

    auto inexactShape = approximate(element.region->shape);

    inexactShape = cartocrow::transform(transformation, inexactShape);

    element.region->shape = pretendExact(inexactShape);

    const Rect oldBox = *element.bb;

    element.bb = Rect{
        oldBox.vertex(0) + translation,
        oldBox.vertex(2) + translation
    };

    element.position = element.position + translation;
}

void ChoroplethMap::translateComponent(const size_t componentIndex, const Vec &delta) {
    MapComponent& component = mapComponents.at(componentIndex);
    translateComponent(component, delta);
}

void ChoroplethMap::translateComponent(MapComponent& component, const Vec &delta) {
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
        if (!element.region || !element.bb) continue;

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

optional<Rect> ChoroplethMap::componentBoundingBox(const MapComponent& component) const {
    optional<Rect> result;

    for (const size_t elementIndex : component.members) {
        const auto& element = mapElements[elementIndex];
        if (!element.bb) continue;

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
    if (!m_map) return;

    renderer.setMode(Renderer::fill | Renderer::stroke);

    auto& relVertices = m_REL->getVertices();
    auto& mapElements = m_map->mapElements;
    for (size_t i = 4; i < relVertices.size(); i++) {
        if (!relVertices[i].isLandRegion) continue;
        if (!mapElements[i].region) continue;

        renderer.setFill(mapElements[i].fillColor);
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
