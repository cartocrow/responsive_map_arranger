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


#include "layout_arranger.h"

LayoutGuide LayoutArranger::computeFittingLayoutGuide(const Rect &container) {

    const auto vertices = m_refLayout.getVertices();

    for (int i = 4; i < vertices.size(); i++) {
        m_refLayout.scaleRelativeVertexSizes(i, container.area());
    }

    const auto longestHorizontalPath = getLongestHorizontalPath(m_refLayout);
    const auto longestVerticalPath = getLongestVerticalPath(m_refLayout);

    const double containerWidth = container.xmax() - container.xmin();
    const double containerHeight = container.ymax() - container.ymin();

    const double horizontalThreshHold = containerWidth + m_mergeSlack * containerWidth;
    const double verticalThreshHold = containerHeight + m_mergeSlack * containerHeight;
    const double horizontalStress = longestHorizontalPath.first - horizontalThreshHold;
    const double verticalStress = longestVerticalPath.first - verticalThreshHold;

    if (horizontalStress > 0 || verticalStress > 0) {
        m_adjustedLayout = LayoutGuide(m_refLayout.getVertices(), m_refLayout.getHalfEdges());
    } else {
        return m_refLayout;
    }

    if (verticalStress >= horizontalStress) {
        // while (verticalStress > 0) TODO: Implement merge
    }
    else {
        // while (horizontalStress > 0) todo: Implement merge
    }

    return m_adjustedLayout;
}

pair<double, vector<int>> LayoutArranger::getLongestHorizontalPath(const LayoutGuide &layoutGuide) const {
    return layoutGuide.getLongestPath(
        HORIZONTAL,
        WEST,
        EAST,
        [&](const int id)->double { return m_refLayout.getVertices()[id].width; },
        4);
}

pair<double, vector<int>> LayoutArranger::getLongestVerticalPath(const LayoutGuide &layoutGuide) const {
    return layoutGuide.getLongestPath(
        VERTICAL,
        SOUTH,
        NORTH,
        [&](const int id)->double { return m_refLayout.getVertices()[id].height; },
        4);
}
