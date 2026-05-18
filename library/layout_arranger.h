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

#include "layout_guide.h"

using namespace cartocrow::layout_guide;

enum MergeHeuristic {
    MIN_CHANGE,
    MIN_WIDTH
};

class LayoutArranger {
    using Rect = cartocrow::Rectangle<cartocrow::Inexact>;
    LayoutArranger(const LayoutGuide &refLayout) : m_refLayout(refLayout), m_adjustedLayout(refLayout) { }

    LayoutGuide computeFittingLayoutGuide(const Rect &container);

    void setMergeHeuristic(const MergeHeuristic heuristic) { m_mergeHeuristic = heuristic; };
    void setMergeSlack(const double slack) { m_mergeSlack = slack; }

private:
    LayoutGuide m_refLayout;
    LayoutGuide m_adjustedLayout;

    MergeHeuristic m_mergeHeuristic = MIN_CHANGE;
    double m_mergeSlack = 0.0;

    pair<double, vector<int>> getLongestHorizontalPath(const LayoutGuide& layoutGuide) const;
    pair<double, vector<int>> getLongestVerticalPath(const LayoutGuide& layoutGuide) const;
};