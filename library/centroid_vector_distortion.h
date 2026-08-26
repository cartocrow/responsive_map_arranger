#pragma once

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <cartocrow/core/core.h>

#include "demers.h"
#include "dorling.h"
#include "choropleth_map.h"
#include "rectangular_dual.h"
#include "regular_edge_labeling.h"

struct GeographicAdjacency {
    std::string firstRegionId;
    std::string secondRegionId;
};

using RegionCentroidMap = std::unordered_map<std::string, cartocrow::Point<cartocrow::Inexact>>;

std::vector<GeographicAdjacency> geographicAdjacenciesFromREL(const RegularEdgeLabeling &rel);
RegionCentroidMap regionCentroids(const cartocrow::RegionMap &regionMap);
RegionCentroidMap rectangularRegionCentroids(const RectangularDual &dual, const RegularEdgeLabeling &rel);
RegionCentroidMap demersRegionCentroids(const DemersCartogram &cartogram);
RegionCentroidMap dorlingRegionCentroids(const DorlingCartogram &cartogram);
RegionCentroidMap choroplethRegionCentroids(const ChoroplethMap &cartogram);

struct LocalDistortionMetrics {
    double deformK = std::numeric_limits<double>::quiet_NaN();
    double orthoOrderK = std::numeric_limits<double>::quiet_NaN();
    double distanceScaleFactor = std::numeric_limits<double>::quiet_NaN();
    std::size_t validSiteCount = 0;
    std::size_t validPairCount = 0;
    std::size_t orthogonalViolationCount = 0;
    std::size_t orthogonalConstraintCount = 0;
};

struct OctantDistortionMetrics {
    std::size_t matchedRegionCount = 0;
    std::size_t comparedPairCount = 0;
    std::size_t changedPairCount = 0;
    std::size_t totalCircularOctantShift = 0;
    double averageCircularOctantShift = std::numeric_limits<double>::quiet_NaN();
};

LocalDistortionMetrics localDistortionMetrics(
    const RegionCentroidMap &baselineCentroids,
    const RegionCentroidMap &adaptiveCentroids,
    std::size_t neighborCount);

OctantDistortionMetrics octantDistortionMetrics(
    const RegionCentroidMap &baselineCentroids,
    const RegionCentroidMap &adaptiveCentroids);
