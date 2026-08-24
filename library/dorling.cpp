#include "dorling.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "rectangular_dual.h"

namespace {

using Inexact = cartocrow::Inexact;
using Pt = cartocrow::Point<Inexact>;

constexpr double kPi = 3.14159265358979323846;

bool hasPositiveExtent(const BoundingBox &bb) {
    return bb.width() > 0.0 && bb.height() > 0.0;
}

Pt stretchPointToBoundingBox(const Pt &point, const BoundingBox &from, const BoundingBox &to) {
    const double xRatio = (point.x() - from.left) / from.width();
    const double yRatio = (point.y() - from.bottom) / from.height();
    return Pt(to.left + xRatio * to.width(), to.bottom + yRatio * to.height());
}

}

void DorlingCartogram::setSourceMapCentroids(const std::unordered_map<std::string, Pt> &centroids,
                                             const BoundingBox &boundingBox) {
    m_sourceMapCentroids = centroids;
    m_sourceMapBoundingBox = boundingBox;
    m_hasSourceMapBoundingBox = hasPositiveExtent(boundingBox);
}

double DorlingCartogram::baseRadiusOf(const Vertex &vertex) const {
    if (!vertex.isLandRegion || vertex.weight <= 0.0) return 0.0;
    return std::sqrt(vertex.weight / kPi);
}

void DorlingCartogram::normalizedDirection(const NodeState &from,
                                           const NodeState &to,
                                           double &dx,
                                           double &dy,
                                           double &distance) const {
    dx = to.x - from.x;
    dy = to.y - from.y;
    distance = std::hypot(dx, dy);
    if (distance < 1e-9) {
        dx = 1e-6;
        dy = 0.0;
        distance = 1e-6;
    }
}

void DorlingCartogram::applyAdjacencyForces(const std::vector<std::pair<int, int>> &adjacencyPairs,
                                            const std::vector<NodeState> &nodes,
                                            std::vector<double> &deltaX,
                                            std::vector<double> &deltaY) const {
    for (const auto &[a, b] : adjacencyPairs) {
        const auto &first = nodes[a];
        const auto &second = nodes[b];

        double dx, dy, distance;
        normalizedDirection(first, second, dx, dy, distance);

        const double targetDistance = first.radius + second.radius + adjacencyPadding;
        if (distance <= targetDistance) continue;

        const double force = std::min(adjacencyForce * (distance - targetDistance), maxAdjacencyForce);
        const double ux = dx / distance;
        const double uy = dy / distance;

        deltaX[a] += force * ux;
        deltaY[a] += force * uy;
        deltaX[b] -= force * ux;
        deltaY[b] -= force * uy;
    }
}

void DorlingCartogram::applyOverlapForces(const std::vector<NodeState> &nodes,
                                          std::vector<double> &deltaX,
                                          std::vector<double> &deltaY) const {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        for (std::size_t j = i + 1; j < nodes.size(); ++j) {
            const auto &first = nodes[i];
            const auto &second = nodes[j];

            double dx, dy, distance;
            normalizedDirection(first, second, dx, dy, distance);

            const double minimumDistance = first.radius + second.radius;
            const double overlap = minimumDistance - distance;
            if (overlap <= 0.0001) continue;

            const double ux = dx / distance;
            const double uy = dy / distance;
            const double push = overlapForce * overlap;

            deltaX[i] -= push * ux;
            deltaY[i] -= push * uy;
            deltaX[j] += push * ux;
            deltaY[j] += push * uy;
        }
    }
}

void DorlingCartogram::applyAnchorForces(const std::vector<NodeState> &nodes,
                                         std::vector<double> &deltaX,
                                         std::vector<double> &deltaY) const {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const auto &node = nodes[i];
        deltaX[i] += (node.anchorX - node.x) * anchorForce;
        deltaY[i] += (node.anchorY - node.y) * anchorForce;
    }
}

double DorlingCartogram::averageRadius(const std::vector<NodeState> &nodes) const {
    if (nodes.empty()) return 0.0;

    double radiusSum = 0.0;
    for (const auto &node : nodes) {
        radiusSum += node.radius;
    }
    return radiusSum / static_cast<double>(nodes.size());
}

double DorlingCartogram::iterationStepScale(const int iteration) const {
    if (forceIterationCount <= 1) return minimumStepScale;

    const double t = static_cast<double>(iteration) / static_cast<double>(forceIterationCount - 1);
    return initialStepScale + (minimumStepScale - initialStepScale) * t;
}

void DorlingCartogram::applyForcesAndClamp(std::vector<NodeState> &nodes,
                                           const std::vector<double> &deltaX,
                                           const std::vector<double> &deltaY,
                                           const BoundingBox &bb,
                                           const double maxStep,
                                           const double stepScale) const {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        auto &node = nodes[i];
        double stepX = deltaX[i] * stepScale;
        double stepY = deltaY[i] * stepScale;
        const double stepLength = std::hypot(stepX, stepY);
        if (stepLength > maxStep && stepLength > 0.0) {
            const double clampScale = maxStep / stepLength;
            stepX *= clampScale;
            stepY *= clampScale;
        }

        node.x += stepX;
        node.y += stepY;

        node.x = std::clamp(node.x, bb.left + node.radius + boundaryPadding, bb.right - node.radius - boundaryPadding);
        node.y = std::clamp(node.y, bb.bottom + node.radius + boundaryPadding, bb.top - node.radius - boundaryPadding);
    }
}

void DorlingCartogram::setFromREL(RegularEdgeLabeling &rel) {
    assert(rel.hasBoundingBox());

    const auto bb = rel.getBoundingBox().value();
    m_box = Rect(bb.left, bb.bottom, bb.right, bb.top);
    m_positions.clear();

    const auto &vertices = rel.getVertices();
    if (vertices.size() <= 4) return;

    const bool useMapCentroids =
        initializationFromMapCentroids &&
        !rel.adaptiveLayoutEnabled() &&
        m_hasSourceMapBoundingBox &&
        !m_sourceMapCentroids.empty();

    std::shared_ptr<RegularEdgeLabeling> relPtr;
    std::unique_ptr<RectangularDual> dual;
    if (!useMapCentroids) {
        relPtr = std::shared_ptr<RegularEdgeLabeling>(&rel, [](RegularEdgeLabeling *) {});
        dual = std::make_unique<RectangularDual>(relPtr);
        dual->setFromREL();
    }

    std::vector<NodeState> nodes;
    std::vector<int> vertexToNode(vertices.size(), -1);
    nodes.reserve(vertices.size() - 4);
    double maxBaseRadius = 0.0;

    for (std::size_t i = 4; i < vertices.size(); ++i) {
        if (!rel.isValidVertex(static_cast<int>(i))) continue;

        Pt center;
        bool hasCenter = false;
        if (useMapCentroids) {
            const auto centroidIt = m_sourceMapCentroids.find(vertices[i].label);
            if (centroidIt != m_sourceMapCentroids.end()) {
                center = stretchPointToBoundingBox(centroidIt->second, m_sourceMapBoundingBox, bb);
                hasCenter = true;
            }
        }
        if (!hasCenter) {
            const auto &rect = dual->getRect(static_cast<std::uint32_t>(i));
            center = rect.center();
        }

        NodeState node;
        node.vertexIndex = static_cast<int>(i);
        node.x = center.x();
        node.y = center.y();
        node.anchorX = center.x();
        node.anchorY = center.y();
        node.radius = baseRadiusOf(vertices[i]);
        maxBaseRadius = std::max(maxBaseRadius, node.radius);

        vertexToNode[i] = static_cast<int>(nodes.size());
        nodes.push_back(node);
    }

    const double areaFractionScale = std::sqrt(std::clamp(targetAreaFraction, 0.0, 1.0));
    double fitScale = areaFractionScale;
    if (maxBaseRadius > 0.0) {
        const double maxAllowedRadius =
            0.5 * std::max(0.0, std::min(bb.width(), bb.height()) - 2.0 * boundaryPadding);
        fitScale = std::min(fitScale, maxAllowedRadius / maxBaseRadius);
    }
    fitScale = std::max(0.0, fitScale);

    for (auto &node : nodes) {
        node.radius *= fitScale;
    }

    const double maxStep = averageRadius(nodes) * maxStepRadiusFraction;

    std::vector<std::pair<int, int>> adjacencyPairs;
    std::unordered_set<std::uint64_t> seenPairs;
    const auto &halfEdges = rel.getHalfEdges();
    for (std::size_t halfEdgeIndex = 0; halfEdgeIndex < halfEdges.size(); ++halfEdgeIndex) {
        const int he = static_cast<int>(halfEdgeIndex);
        if (!rel.isValidHalfEdge(he)) continue;

        const auto &halfEdge = halfEdges[halfEdgeIndex];
        if (!halfEdge.outgoing || halfEdge.color == BLACK) continue;

        const int source = halfEdge.vertex;
        const int target = rel.neighborOfHalfEdge(he);
        if (!rel.isValidVertex(source) || !rel.isValidVertex(target)) continue;
        if (!rel.isInnerVertex(source) || !rel.isInnerVertex(target)) continue;
        if (!vertices[source].isLandRegion || !vertices[target].isLandRegion) continue;

        const int sourceNode = vertexToNode[source];
        const int targetNode = vertexToNode[target];
        if (sourceNode < 0 || targetNode < 0 || sourceNode == targetNode) continue;

        const int a = std::min(sourceNode, targetNode);
        const int b = std::max(sourceNode, targetNode);
        const std::uint64_t key = (static_cast<std::uint64_t>(a) << 32U) | static_cast<std::uint32_t>(b);
        if (!seenPairs.insert(key).second) continue;
        adjacencyPairs.emplace_back(a, b);
    }

    std::vector<double> deltaX(nodes.size(), 0.0);
    std::vector<double> deltaY(nodes.size(), 0.0);

    for (int iteration = 0; iteration < forceIterationCount; ++iteration) {
        std::fill(deltaX.begin(), deltaX.end(), 0.0);
        std::fill(deltaY.begin(), deltaY.end(), 0.0);

        applyAdjacencyForces(adjacencyPairs, nodes, deltaX, deltaY);
        applyOverlapForces(nodes, deltaX, deltaY);
        applyAnchorForces(nodes, deltaX, deltaY);
        const double stepScale = iterationStepScale(iteration);
        applyForcesAndClamp(nodes, deltaX, deltaY, bb, maxStep, stepScale);
    }

    m_positions.reserve(nodes.size());
    for (const auto &node : nodes) {
        m_positions.emplace_back(vertices[node.vertexIndex], node.x, node.y, node.radius);
    }
}

void DorlingPainting::paint(Renderer &renderer) const {
    if (!m_cartogram) return;

    renderer.setMode(Renderer::fill | Renderer::stroke);
    renderer.setStroke({32, 32, 32}, 1.0);

    for (const auto &position : m_cartogram->m_positions) {
        if (position.radius <= 0.0) continue;
        renderer.setFill(position.color);
        renderer.draw(cartocrow::Circle<Inexact>(position.center, position.radius * position.radius));

        if (m_drawLabels) {
            renderer.setFill({0, 0, 0});
            renderer.drawText(position.center, position.label);
        }
    }

    renderer.setStroke({102, 102, 102}, 2.0);
    renderer.setMode(Renderer::stroke);
    renderer.draw(m_cartogram->m_box);
}
