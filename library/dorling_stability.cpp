#include "dorling_stability.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {

using Pt = DorlingCartogram::Pt;

Pt containerCenter(const BoundingBox &box) {
    return Pt(0.5 * (box.left + box.right), 0.5 * (box.bottom + box.top));
}

}

DorlingStabilityResult dorlingStability(
    const DorlingCartogram &layoutA,
    const BoundingBox &containerA,
    const DorlingCartogram &layoutB,
    const BoundingBox &containerB) {
    DorlingStabilityResult result;

    std::unordered_map<std::string, Pt> centersA;
    centersA.reserve(layoutA.positions().size());
    for (const auto &position : layoutA.positions()) {
        if (position.label.empty()) continue;
        if (position.label.starts_with("sea_")) continue;
        centersA[position.label] = position.center;
    }

    struct MatchedCenter {
        std::string label;
        Pt a;
        Pt b;
    };

    std::vector<MatchedCenter> matchedCenters;
    matchedCenters.reserve(std::min(layoutA.positions().size(), layoutB.positions().size()));
    for (const auto &position : layoutB.positions()) {
        if (position.label.empty()) continue;
        if (position.label.starts_with("sea_")) continue;

        const auto it = centersA.find(position.label);
        if (it == centersA.end()) continue;
        matchedCenters.push_back({position.label, it->second, position.center});
    }

    result.matchedRegionCount = matchedCenters.size();
    if (matchedCenters.empty()) {
        result.rms = std::numeric_limits<double>::quiet_NaN();
        result.optimalScale = std::numeric_limits<double>::quiet_NaN();
        return result;
    }

    std::sort(matchedCenters.begin(), matchedCenters.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.label < rhs.label;
    });

    const Pt centerA = containerCenter(containerA);
    const Pt centerB = containerCenter(containerB);

    double numerator = 0.0;
    double denominator = 0.0;
    for (const auto &matched : matchedCenters) {
        const double ax = matched.a.x() - centerA.x();
        const double ay = matched.a.y() - centerA.y();
        const double bx = matched.b.x() - centerB.x();
        const double by = matched.b.y() - centerB.y();

        numerator += ax * bx + ay * by;
        denominator += bx * bx + by * by;
    }

    result.optimalScale = denominator > 0.0 ? numerator / denominator : 0.0;

    double squaredErrorSum = 0.0;
    for (const auto &matched : matchedCenters) {
        const double ax = matched.a.x() - centerA.x();
        const double ay = matched.a.y() - centerA.y();
        const double bx = matched.b.x() - centerB.x();
        const double by = matched.b.y() - centerB.y();

        const double dx = ax - result.optimalScale * bx;
        const double dy = ay - result.optimalScale * by;
        squaredErrorSum += dx * dx + dy * dy;
    }

    result.rms = std::sqrt(squaredErrorSum / static_cast<double>(matchedCenters.size()));
    return result;
}
