//
// Created by arjen on 4/8/26.
//

#include "layout_guide.h"

namespace cartocrow::layout_guide {
LayoutGuide::LayoutGuide(vector<LayoutElement> layoutElements,
                         vector<HalfEdge> halfEdges)
                        : m_elements(layoutElements), m_halfEdges(halfEdges) {
}

// returns heID if heID is outgoing and returns its twin if heID is not outgoing
int LayoutGuide::getCanonicalHalfEdge(int const &heID) const {
    if (!isValidHalfEdge(heID)) return -1;

    const HalfEdge &he = m_halfEdges[heID];
    if (he.m_outgoing) return heID;
    const int twinID = he.m_twin;
    if (!isValidHalfEdge(twinID)) return -1;
    return twinID;

}

// int RegularEdgeLabeling::canonicalHalfEdge(int he) const {
//     if (he < 0 || he >= (int)m_halfEdges.size()) return -1;
//     const HalfEdge &h = m_halfEdges[he];
//     if (h.outgoing) return he;
//     int t = h.twin;
//     if (t >= 0 && t < (int)m_halfEdges.size() && m_halfEdges[t].outgoing) return t;
//     return he;
// }
} // namespace cartocrow::layout_guide
