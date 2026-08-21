#include "centroid_vector_distortion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <utility>

#include <cartocrow/core/polygon_helpers.h>

namespace {

std::pair<std::string, std::string> canonicalPair(const std::string &a, const std::string &b) {
    if (a <= b) return {a, b};
    return {b, a};
}

int axisRelation(const double delta) {
    constexpr double kEpsilon = 1e-9;
    if (delta > kEpsilon) return 1;
    if (delta < -kEpsilon) return -1;
    return 0;
}

double squaredDistance(
    const cartocrow::Point<cartocrow::Inexact> &a,
    const cartocrow::Point<cartocrow::Inexact> &b) {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    return dx * dx + dy * dy;
}

}

std::vector<GeographicAdjacency> geographicAdjacenciesFromREL(const RegularEdgeLabeling &rel) {
    std::vector<GeographicAdjacency> adjacencies;
    std::set<std::pair<std::string, std::string>> seenPairs;

    const auto &vertices = rel.getVertices();
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
        if (!rel.isLandVertex(source) || !rel.isLandVertex(target)) continue;
        if (source == target) continue;

        const auto &sourceId = vertices[source].label;
        const auto &targetId = vertices[target].label;
        if (sourceId.empty() || targetId.empty() || sourceId == targetId) continue;

        const auto pair = canonicalPair(sourceId, targetId);
        if (!seenPairs.insert(pair).second) continue;

        adjacencies.push_back({pair.first, pair.second});
    }

    return adjacencies;
}

RegionCentroidMap regionCentroids(const cartocrow::RegionMap &regionMap) {
    RegionCentroidMap centroids;

    for (const auto &[label, region] : regionMap) {
        if (label.empty()) continue;
        centroids[label] = cartocrow::centroid(cartocrow::approximate(region.shape));
    }

    return centroids;
}

RegionCentroidMap rectangularRegionCentroids(const RectangularDual &dual, const RegularEdgeLabeling &rel) {
    RegionCentroidMap centroids;

    const auto &vertices = rel.getVertices();
    const auto &rectangles = dual.rectangles();

    const std::size_t count = std::min(vertices.size(), rectangles.size());
    for (std::size_t i = 0; i < count; ++i) {
        const int vertexIndex = static_cast<int>(i);
        if (!rel.isValidVertex(vertexIndex) || !rel.isInnerVertex(vertexIndex)) continue;
        if (!rel.isLandVertex(vertexIndex)) continue;

        const auto &rect = rectangles[i];
        if (rect.isDisabled) continue;

        centroids[vertices[i].label] = rect.center();
    }

    return centroids;
}

RegionCentroidMap demersRegionCentroids(const DemersCartogram &cartogram) {
    RegionCentroidMap centroids;

    for (const auto &position : cartogram.positions()) {
        if (position.label.empty()) continue;
        if (position.label.starts_with("sea_")) continue;
        centroids[position.label] = position.center;
    }

    return centroids;
}

RegionCentroidMap dorlingRegionCentroids(const DorlingCartogram &cartogram) {
    RegionCentroidMap centroids;

    for (const auto &position : cartogram.positions()) {
        if (position.label.empty()) continue;
        if (position.label.starts_with("sea_")) continue;
        centroids[position.label] = position.center;
    }

    return centroids;
}

LocalDistortionMetrics localDistortionMetrics(
    const RegionCentroidMap &baselineCentroids,
    const RegionCentroidMap &adaptiveCentroids,
    std::size_t neighborCount) {
    LocalDistortionMetrics metrics;

    struct MatchedRegion {
        std::string label;
        cartocrow::Point<cartocrow::Inexact> baseline;
        cartocrow::Point<cartocrow::Inexact> adaptive;
    };

    std::vector<MatchedRegion> matchedRegions;
    matchedRegions.reserve(baselineCentroids.size());

    for (const auto &[label, baselinePoint] : baselineCentroids) {
        const auto adaptiveIt = adaptiveCentroids.find(label);
        if (adaptiveIt == adaptiveCentroids.end()) continue;
        matchedRegions.push_back({label, baselinePoint, adaptiveIt->second});
    }

    metrics.validSiteCount = matchedRegions.size();
    if (metrics.validSiteCount < 2 || neighborCount == 0) return metrics;

    std::sort(matchedRegions.begin(), matchedRegions.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.label < rhs.label;
    });

    const std::size_t effectiveNeighborCount = std::min(neighborCount, matchedRegions.size() - 1);

    double baselineDistanceSquaredSum = 0.0;
    double pairedDistanceProductSum = 0.0;
    std::vector<std::pair<double, double>> pairedDistances;

    for (std::size_t i = 0; i < matchedRegions.size(); ++i) {
        std::vector<std::tuple<double, std::string, std::size_t>> neighbors;
        neighbors.reserve(matchedRegions.size() - 1);

        for (std::size_t j = 0; j < matchedRegions.size(); ++j) {
            if (i == j) continue;
            neighbors.emplace_back(
                squaredDistance(matchedRegions[i].baseline, matchedRegions[j].baseline),
                matchedRegions[j].label,
                j);
        }

        std::sort(neighbors.begin(), neighbors.end(), [](const auto &lhs, const auto &rhs) {
            if (std::get<0>(lhs) != std::get<0>(rhs)) return std::get<0>(lhs) < std::get<0>(rhs);
            return std::get<1>(lhs) < std::get<1>(rhs);
        });

        for (std::size_t neighborIndex = 0; neighborIndex < effectiveNeighborCount; ++neighborIndex) {
            const auto j = std::get<2>(neighbors[neighborIndex]);

            const double baselineDx = matchedRegions[j].baseline.x() - matchedRegions[i].baseline.x();
            const double baselineDy = matchedRegions[j].baseline.y() - matchedRegions[i].baseline.y();
            const double adaptiveDx = matchedRegions[j].adaptive.x() - matchedRegions[i].adaptive.x();
            const double adaptiveDy = matchedRegions[j].adaptive.y() - matchedRegions[i].adaptive.y();

            const double baselineDistance = std::hypot(baselineDx, baselineDy);
            const double adaptiveDistance = std::hypot(adaptiveDx, adaptiveDy);

            baselineDistanceSquaredSum += baselineDistance * baselineDistance;
            pairedDistanceProductSum += baselineDistance * adaptiveDistance;
            pairedDistances.emplace_back(baselineDistance, adaptiveDistance);

            if (axisRelation(baselineDx) != axisRelation(adaptiveDx)) ++metrics.orthogonalViolationCount;
            if (axisRelation(baselineDy) != axisRelation(adaptiveDy)) ++metrics.orthogonalViolationCount;
        }
    }

    metrics.validPairCount = pairedDistances.size();
    metrics.orthogonalConstraintCount = 2 * metrics.validPairCount;

    if (metrics.validPairCount == 0 || baselineDistanceSquaredSum == 0.0) return metrics;

    metrics.distanceScaleFactor = pairedDistanceProductSum / baselineDistanceSquaredSum;

    double residualSquaredSum = 0.0;
    for (const auto &[baselineDistance, adaptiveDistance] : pairedDistances) {
        const double residual = metrics.distanceScaleFactor * baselineDistance - adaptiveDistance;
        residualSquaredSum += residual * residual;
    }

    metrics.deformK = std::sqrt(residualSquaredSum / static_cast<double>(metrics.validPairCount));
    if (metrics.orthogonalConstraintCount > 0) {
        metrics.orthoOrderK = static_cast<double>(metrics.orthogonalViolationCount) /
                              static_cast<double>(metrics.orthogonalConstraintCount);
    }

    return metrics;
}
