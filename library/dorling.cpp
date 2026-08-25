#include "dorling.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <queue>

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

double clampPositiveSize(const double value) {
    return (std::isfinite(value) && value > 0.0) ? value : 1.0;
}

template <typename SizeFn>
std::unordered_map<int, double> computeAxisCoordinates(const std::vector<int> &vertexIds,
                                                       const std::vector<Vertex> &vertices,
                                                       const double axisMin,
                                                       const double axisMax,
                                                       SizeFn sizeOf) {
    std::unordered_map<int, double> positions;
    if (vertexIds.empty()) return positions;

    const double span = axisMax - axisMin;
    if (vertexIds.size() == 1 || span <= 0.0) {
        positions[vertexIds.front()] = axisMin + 0.5 * span;
        return positions;
    }

    double totalSize = 0.0;
    for (const int vertexId : vertexIds) {
        totalSize += clampPositiveSize(sizeOf(vertices[vertexId]));
    }

    if (totalSize <= 0.0) {
        const double denom = static_cast<double>(vertexIds.size() - 1);
        for (std::size_t i = 0; i < vertexIds.size(); ++i) {
            positions[vertexIds[i]] = axisMin + span * (static_cast<double>(i) / denom);
        }
        return positions;
    }

    double consumed = 0.0;
    for (const int vertexId : vertexIds) {
        const double size = clampPositiveSize(sizeOf(vertices[vertexId]));
        positions[vertexId] = axisMin + span * ((consumed + 0.5 * size) / totalSize);
        consumed += size;
    }

    return positions;
}

struct WeightedEdge {
    int target = -1;
    double weight = 0.0;
};

bool topoSortWeighted(const std::vector<std::vector<WeightedEdge>> &adj, std::vector<int> &order) {
    order.clear();
    std::vector<int> indegree(adj.size(), 0);
    for (const auto &neighbors : adj) {
        for (const auto &edge : neighbors) {
            if (edge.target >= 0 && edge.target < static_cast<int>(adj.size())) {
                ++indegree[edge.target];
            }
        }
    }

    std::queue<int> queue;
    for (int i = 0; i < static_cast<int>(adj.size()); ++i) {
        if (indegree[i] == 0) queue.push(i);
    }

    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        order.push_back(node);

        for (const auto &edge : adj[node]) {
            if (--indegree[edge.target] == 0) {
                queue.push(edge.target);
            }
        }
    }

    return order.size() == adj.size();
}

template <typename EdgeWeightFn>
std::unordered_map<int, double> computeSegmentCoordinates(const RegularEdgeLabeling &rel,
                                                          const bool horizontalAxis,
                                                          const double axisMin,
                                                          const double axisMax,
                                                          EdgeWeightFn edgeWeight) {
    const auto &vertices = rel.getVertices();

    std::vector<int> segmentIds;
    segmentIds.reserve(vertices.size() * 2);
    for (int v = 0; v < static_cast<int>(vertices.size()); ++v) {
        if (!rel.isValidVertex(v)) continue;

        const int first = horizontalAxis ? vertices[v].left_segment : vertices[v].bottom_segment;
        const int second = horizontalAxis ? vertices[v].right_segment : vertices[v].top_segment;
        if (first >= 0) segmentIds.push_back(first);
        if (second >= 0) segmentIds.push_back(second);
    }

    std::sort(segmentIds.begin(), segmentIds.end());
    segmentIds.erase(std::unique(segmentIds.begin(), segmentIds.end()), segmentIds.end());

    std::unordered_map<int, int> segToNode;
    segToNode.reserve(segmentIds.size());
    for (std::size_t i = 0; i < segmentIds.size(); ++i) {
        segToNode.emplace(segmentIds[i], static_cast<int>(i));
    }

    const int frameStart = static_cast<int>(segmentIds.size());
    const int frameEnd = frameStart + 1;
    std::vector<std::unordered_map<int, double>> edgeMap(segmentIds.size() + 2);

    for (int v = 0; v < static_cast<int>(vertices.size()); ++v) {
        if (!rel.isValidVertex(v)) continue;

        const int firstSeg = horizontalAxis ? vertices[v].left_segment : vertices[v].bottom_segment;
        const int secondSeg = horizontalAxis ? vertices[v].right_segment : vertices[v].top_segment;

        const int from = firstSeg >= 0 ? segToNode.at(firstSeg) : frameStart;
        const int to = secondSeg >= 0 ? segToNode.at(secondSeg) : frameEnd;
        if (from == to) continue;

        const double weight = rel.isInnerVertex(v) ? clampPositiveSize(edgeWeight(vertices[v])) : 0.0;
        auto [it, inserted] = edgeMap[from].emplace(to, weight);
        if (!inserted) {
            it->second = std::max(it->second, weight);
        }
    }

    std::vector<std::vector<WeightedEdge>> adjacency(edgeMap.size());
    for (int node = 0; node < static_cast<int>(edgeMap.size()); ++node) {
        adjacency[node].reserve(edgeMap[node].size());
        for (const auto &[target, weight] : edgeMap[node]) {
            adjacency[node].push_back(WeightedEdge{target, weight});
        }
    }

    std::vector<int> topoOrder;
    if (!topoSortWeighted(adjacency, topoOrder)) {
        return {};
    }

    std::vector<double> distance(adjacency.size(), 0.0);
    for (const int node : topoOrder) {
        for (const auto &edge : adjacency[node]) {
            distance[edge.target] = std::max(distance[edge.target], distance[node] + edge.weight);
        }
    }

    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    for (int node = 0; node < static_cast<int>(segmentIds.size()); ++node) {
        minValue = std::min(minValue, distance[node]);
        maxValue = std::max(maxValue, distance[node]);
    }

    if (!std::isfinite(minValue) || !std::isfinite(maxValue)) {
        minValue = 0.0;
        maxValue = 1.0;
    } else if (maxValue == minValue) {
        maxValue = minValue + 1.0;
    }

    const double span = axisMax - axisMin;
    std::unordered_map<int, double> coordinates;
    coordinates.reserve(segmentIds.size());
    for (std::size_t i = 0; i < segmentIds.size(); ++i) {
        const double t = (distance[static_cast<int>(i)] - minValue) / (maxValue - minValue);
        coordinates.emplace(segmentIds[i], axisMin + t * span);
    }

    return coordinates;
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
        if (distance <= targetDistance*0.5) continue;

        const double force = adjacencyForce * (distance-targetDistance);// std::min(adjacencyForce * (distance - targetDistance), maxAdjacencyForce);
        const double ux = dx / distance;
        const double uy = dy / distance;

        deltaX[a] += force * ux;
        deltaY[a] += force * uy;
        deltaX[b] -= force * ux;
        deltaY[b] -= force * uy;
    }
}

void DorlingCartogram::applyRELDirectionalForces(const RegularEdgeLabeling &rel,
                                                 const std::vector<int> &vertexToNode,
                                                 const std::vector<NodeState> &nodes,
                                                 std::vector<double> &deltaX,
                                                 std::vector<double> &deltaY) const {
    if (!rel.adaptiveLayoutEnabled() || relDirectionalForce <= 0.0) return;

    const auto &halfEdges = rel.getHalfEdges();
    const auto &vertices = rel.getVertices();

    for (std::size_t halfEdgeIndex = 0; halfEdgeIndex < halfEdges.size(); ++halfEdgeIndex) {
        const auto &halfEdge = halfEdges[halfEdgeIndex];
        if (halfEdge.isDeleted || !halfEdge.outgoing || halfEdge.color == BLACK) continue;

        const int source = halfEdge.vertex;
        const int target = rel.neighborOfHalfEdge(static_cast<int>(halfEdgeIndex));
        if (!rel.isValidVertex(source) || !rel.isValidVertex(target)) continue;
        if (!rel.isInnerVertex(source) || !rel.isInnerVertex(target)) continue;
        if (!vertices[source].isLandRegion || !vertices[target].isLandRegion) continue;

        const int sourceNode = vertexToNode[source];
        const int targetNode = vertexToNode[target];
        if (sourceNode < 0 || targetNode < 0 || sourceNode == targetNode) continue;

        const auto &sourceState = nodes[sourceNode];
        const auto &targetState = nodes[targetNode];

        if (halfEdge.color == BLUE) {
            if (targetState.x - targetState.radius > sourceState.x + sourceState.radius) continue;
            deltaX[sourceNode] -= relDirectionalForce;
            deltaX[targetNode] += relDirectionalForce;
        } else if (halfEdge.color == RED) {
            if (targetState.y - targetState.radius > sourceState.y + sourceState.radius) continue;
            deltaY[sourceNode] -= relDirectionalForce;
            deltaY[targetNode] += relDirectionalForce;
        }
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
            if (overlap <= 0) continue;

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
    if (anchorForce <=0 ) return;
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

std::unordered_map<int, DorlingCartogram::Pt> DorlingCartogram::computeGuideInitializationCenters(
    const RegularEdgeLabeling &rel,
    const BoundingBox &bb,
    const bool weighted) const {
    const auto &vertices = rel.getVertices();

    const auto xSegments = computeSegmentCoordinates(
        rel,
        true,
        bb.left,
        bb.right,
        [weighted](const Vertex &vertex) { return weighted ? vertex.preferred_width : 1.0; });
    const auto ySegments = computeSegmentCoordinates(
        rel,
        false,
        bb.bottom,
        bb.top,
        [weighted](const Vertex &vertex) { return weighted ? vertex.preferred_height : 1.0; });

    std::unordered_map<int, Pt> centers;
    centers.reserve(vertices.size());
    const Pt fallbackCenter{0.5 * (bb.left + bb.right), 0.5 * (bb.bottom + bb.top)};

    for (int i = 4; i < static_cast<int>(vertices.size()); ++i) {
        if (!rel.isValidVertex(i)) continue;
        if (!vertices[i].isLandRegion) continue;

        const auto leftIt = xSegments.find(vertices[i].left_segment);
        const auto rightIt = xSegments.find(vertices[i].right_segment);
        const auto bottomIt = ySegments.find(vertices[i].bottom_segment);
        const auto topIt = ySegments.find(vertices[i].top_segment);

        const double left = leftIt != xSegments.end() ? leftIt->second : bb.left;
        const double right = rightIt != xSegments.end() ? rightIt->second : bb.right;
        const double bottom = bottomIt != ySegments.end() ? bottomIt->second : bb.bottom;
        const double top = topIt != ySegments.end() ? topIt->second : bb.top;

        const double x = std::isfinite(left) && std::isfinite(right) ? 0.5 * (left + right) : fallbackCenter.x();
        const double y = std::isfinite(bottom) && std::isfinite(top) ? 0.5 * (bottom + top) : fallbackCenter.y();
        centers.emplace(i, Pt{x, y});
    }

    return centers;
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
    std::unordered_map<int, Pt> guideCenters;
    if (rel.adaptiveLayoutEnabled() &&
        adaptiveInitializationMode != AdaptiveDorlingInitializationMode::RectangularCartogramCenters) {
        relPtr = std::shared_ptr<RegularEdgeLabeling>(&rel, [](RegularEdgeLabeling *) {});
        dual = std::make_unique<RectangularDual>(relPtr);
        dual->setFromREL();
        guideCenters = computeGuideInitializationCenters(
            rel,
            bb,
            adaptiveInitializationMode == AdaptiveDorlingInitializationMode::LayoutGuideOrderWeighted);
    }

    if (!useMapCentroids && !dual) {
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
            const auto guideIt = guideCenters.find(static_cast<int>(i));
            if (guideIt != guideCenters.end()) {
                center = guideIt->second;
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

    const double maxStep = averageRadius(nodes);// * maxStepRadiusFraction;

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

    const int separationIterations = std::max(1, separationIterationsPerAttraction);
    for (int iteration = 0; iteration < forceIterationCount; ++iteration) {
        const double stepScale = iterationStepScale(iteration);

        for (int separationIteration = 0; separationIteration < separationIterations - 1; ++separationIteration) {
            std::fill(deltaX.begin(), deltaX.end(), 0.0);
            std::fill(deltaY.begin(), deltaY.end(), 0.0);

            applyOverlapForces(nodes, deltaX, deltaY);
            applyForcesAndClamp(nodes, deltaX, deltaY, bb, maxStep, stepScale);
        }

        std::fill(deltaX.begin(), deltaX.end(), 0.0);
        std::fill(deltaY.begin(), deltaY.end(), 0.0);

        applyAdjacencyForces(adjacencyPairs, nodes, deltaX, deltaY);
        applyRELDirectionalForces(rel, vertexToNode, nodes, deltaX, deltaY);
        applyOverlapForces(nodes, deltaX, deltaY);
        applyAnchorForces(nodes, deltaX, deltaY);
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
