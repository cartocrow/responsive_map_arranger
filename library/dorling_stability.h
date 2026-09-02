#pragma once

#include <cstddef>
#include <string>

#include "dorling.h"
#include "geometry_types.h"

struct DorlingStabilityResult {
    double rms = 0.0;
    double optimalScale = 0.0;
    std::size_t matchedRegionCount = 0;
};

DorlingStabilityResult dorlingStability(
    const DorlingCartogram &layoutA,
    const BoundingBox &containerA,
    const DorlingCartogram &layoutB,
    const BoundingBox &containerB);
