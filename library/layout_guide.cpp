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

#include "layout_guide.h"

namespace cartocrow::layout_guide {
LayoutGuide::LayoutGuide(vector<Vertex> vertices,
                         vector<HalfEdge> halfEdges)
                        : m_vertices(vertices), m_halfEdges(halfEdges) {
}

// returns heID if heID is outgoing and returns its twin if heID is not outgoing
int LayoutGuide::getCanonicalHalfEdge(int const &heId) const {
    if (!isValidHalfEdge(heId)) return -1;

    const HalfEdge &he = m_halfEdges[heId];
    if (he.m_outgoing) return heId;
    const int twinID = he.m_twin;
    if (!isValidHalfEdge(twinID)) return -1;
    return twinID;

}

int LayoutGuide::getNextCyclicEdge(int const &heId) const {
    const int pos = getCanonicalHalfEdge(heId);
    const Vertex v = m_vertices[m_halfEdges[heId].m_vertex];

    return v.m_edges[(pos + v.degree() - 1) % v.degree()];
}

int LayoutGuide::getPreviousCyclicEdge(int const &heId) const {
    const int pos = getCanonicalHalfEdge(heId);
    const Vertex v = m_vertices[m_halfEdges[heId].m_vertex];

    return v.m_edges[(pos+1) % v.degree()];
}

int LayoutGuide::getCyclicPositionOfHalfEdge(int const &heId) const {
    if (!isValidHalfEdge(heId)) return -1;

    const Vertex v = m_vertices[m_halfEdges[heId].m_vertex];
    for (int i = 0; i < v.degree(); ++i)
        if (v.m_edges[i] == heId) return i;

    return -1;
}

int LayoutGuide::getFirstEdgeOfType(const int vId, const EdgeLabel edgeLabel, const bool outgoing) const {
    const Vertex &v = m_vertices[vId];

    for (int i = 0; i < v.degree(); i++) {
        const int edgeId = v.m_edges[i];
        const int prevEdgeId = getPreviousCyclicEdge(edgeId);

        const HalfEdge &edge = m_halfEdges[edgeId];
        const HalfEdge &prevEdge = m_halfEdges[prevEdgeId];

        if (edge.m_edgeLabel == edgeLabel && edge.m_outgoing == outgoing && (prevEdge.m_edgeLabel != edgeLabel || prevEdge.m_outgoing != outgoing)) {
            return v.m_edges[i];
        }
    }
    return -1;
}

int LayoutGuide::getLastEdgeOfType(const int vId, const EdgeLabel edgeLabel, const bool outgoing) const {
    const Vertex &v = m_vertices[vId];

    for (int i = 0; i < v.degree(); i++) {
        const int edgeId = v.m_edges[i];
        const int nextEdgeId = getNextCyclicEdge(edgeId);

        const HalfEdge &edge = m_halfEdges[edgeId];
        const HalfEdge &nextEdge = m_halfEdges[nextEdgeId];

        if (edge.m_edgeLabel == edgeLabel && edge.m_outgoing == outgoing && (nextEdge.m_edgeLabel != edgeLabel || nextEdge.m_outgoing != outgoing)) {
            return v.m_edges[i];
        }
    }
    return -1;
}

bool LayoutGuide::flipEdgeColor(int const &heID) {
    if (!isValidHalfEdge(heID)) return false;
    HalfEdge &he = m_halfEdges[heID];
    if (!isValidHalfEdge(he.m_twin)) return false;
    HalfEdge &twin = m_halfEdges[he.m_twin];

    if (he.m_edgeLabel == HORIZONTAL) {
        he.m_edgeLabel = VERTICAL;
        twin.m_edgeLabel = HORIZONTAL;
        return true;
    }
    if (he.m_edgeLabel == VERTICAL) {
        he.m_edgeLabel = HORIZONTAL;
        twin.m_edgeLabel = VERTICAL;
        return true;
    }
    return false;
}

bool LayoutGuide::flipEdgeDiagonally(int const &heId, bool clockwise) {
    const int baseHeId = getCanonicalHalfEdge(heId);
    if (!isValidHalfEdge(baseHeId))
        throw runtime_error("flipEdgeDiagonally: Invalid edgeID: " + to_string(heId));

    HalfEdge &baseHe = m_halfEdges[baseHeId];
    int endHeId = baseHe.m_twin;

    if (!isValidHalfEdge(endHeId))
        throw runtime_error("flipEdgeDiagonally: Invalid edgeID: " + to_string(heId));

    HalfEdge &endHe = m_halfEdges[endHeId];

    const int baseVertexId = baseHe.m_vertex;
    const int endVertexId = endHe.m_vertex;
    Vertex& baseVertex = m_vertices[baseVertexId];
    Vertex& endVertex = m_vertices[endVertexId];

    if (!isValidVertex(baseVertexId) || !isValidVertex(endVertexId)) return false;

    const int basePos = getCyclicPositionOfHalfEdge(baseHeId);
    const int endPos = getCyclicPositionOfHalfEdge(endVertexId);

    // Get new base and en
    int targetBaseHe = -1;
    int targetEndHe = -1;
    if (clockwise) {
        targetBaseHe = getNextCyclicEdge(baseHeId);
        targetEndHe = getPreviousCyclicEdge(baseHeId);

    } else {
        targetBaseHe = getPreviousCyclicEdge(baseHeId);
        targetEndHe = getNextCyclicEdge(baseHeId);
    }

    int targetBaseVertexId = m_halfEdges[m_halfEdges[targetBaseHe].m_twin].m_vertex;
    int targetEndVertexId = m_halfEdges[m_halfEdges[targetEndHe].m_twin].m_vertex;
    Vertex& targetBaseVertex = m_vertices[targetBaseVertexId];
    Vertex& targetEndVertex = m_vertices[targetEndVertexId];

    // erase edges from original vertices edge list
    baseVertex.m_edges.erase(baseVertex.m_edges.begin() + basePos);
    endVertex.m_edges.erase(endVertex.m_edges.begin() + endPos);

    // insert he into target base/end vertices
    int baseInsertPos = getCyclicPositionOfHalfEdge(targetBaseHe);
    if (clockwise) baseInsertPos++;
    baseInsertPos = clamp(baseInsertPos, 0, targetBaseVertex.degree()); // -1 means we insert at the beginning
    targetBaseVertex.m_edges.insert(targetBaseVertex.m_edges.begin() + baseInsertPos, endHeId);

    int endInsertPos = getCyclicPositionOfHalfEdge(targetEndHe);
    if (!clockwise) endInsertPos++;
    endInsertPos = clamp(endInsertPos, 0, targetEndVertex.degree()); // -1 means we insert at the beginning
    targetEndVertex.m_edges.insert(targetEndVertex.m_edges.begin() + endInsertPos, endHeId);

    // update half edge vertex references
    m_halfEdges[baseHeId].m_vertex = targetBaseVertexId;
    m_halfEdges[endHeId].m_vertex = targetEndVertexId;

    return true;
}

bool LayoutGuide::redirectEdge(int const &heID) {
    if (!isValidHalfEdge(heID)) return false;

    HalfEdge &he = m_halfEdges[heID];
    he.m_outgoing = !he.m_outgoing;

    HalfEdge &twin = m_halfEdges[he.m_twin];
    twin.m_outgoing = !twin.m_outgoing;

    return true;
}

bool LayoutGuide::checkRelHalfEdgesConsistency() const {
    for (HalfEdge he : m_halfEdges) {
        const Vertex baseVertex = m_vertices[he.m_vertex];
        if (!isValidHalfEdge(he.m_twin)) {
            std::cerr << "Invalid REL: twin edge " << he.m_twin
            << " is an invalid twin-he id of the he incident to vertex " << baseVertex.m_label << std::endl;
            return false;
        }

        const HalfEdge twin = m_halfEdges[he.m_twin];
        const Vertex endVertex = m_vertices[twin.m_vertex];

        if (he.m_edgeLabel != twin.m_edgeLabel) {
            std::cerr << "Invalid REL: inconsistent edge labels for half edges between "
            << baseVertex.m_label << " and " << endVertex.m_label << endl;
            return false;
        }

        if (he.m_outgoing == twin.m_outgoing) {
            std::cerr << "Invalid REL: inconsistent edge direction for half edges between "
            << baseVertex.m_label << " and " << endVertex.m_label << endl;
            return false;
        }
    }

    return true;
}

bool LayoutGuide::checkCyclicEdgeTypeOrder() const {
    for (Vertex v : m_vertices) {
        if (v.m_edges.empty()) {
            std::cout << "[Warning] vertex " << v.m_label << " has degree 0." << std::endl;
            continue;
        }

        vector<EdgeType> blocks;
        blocks.reserve(v.m_edges.size());

        for (const int heId : v.m_edges) {
            const HalfEdge edge = m_halfEdges[heId];
            const EdgeType type = edge.edgeType();
            if (type == NONE) {
                std::cerr << "Invalid REL: vertex " << v.m_label << " has an incident edge of EdgeType NONE" << std::endl;
                return false;
            }

            // add type to block if the last block type is different
            if (blocks.empty() || blocks.back() != type)
                blocks.push_back(type);

        }

        // remove last block if first and last block are the same
        if (blocks.size() > 1 && blocks.front() == blocks.back())
            blocks.pop_back();

        if (blocks.size() != 4) {
            std::cerr << "Invalid REL: cyclic EdgeType order of " << v.m_label << " is not correct: ";
            for (const auto b : blocks)
                std::cerr << b << " ";
            std::cerr << std::endl;
            return false;
        }

        constexpr std::array<EdgeType, 4> expected = {
            OUTGOING_VERTICAL,
            OUTGOING_HORIZONTAL,
            INCOMING_VERTICAL,
            INCOMING_HORIZONTAL
        };

        bool matches = false;

        // check whether blocks match expected order up to cyclic rotation
        for (int shift = 0; shift < 4; ++shift) {
            matches = true;
            for (int i = 0; i < 4; ++i) {
                if (blocks[i] != expected[(i + shift) % 4]) {
                    matches = false;
                    break;
                }
            }
            if (matches) {
                break;
            }
        }

        if (!matches) {
            std::cerr << "Invalid REL: cyclic EdgeType order of " << v.m_label << " is not correct: ";
            for (const auto b : blocks)
                std::cerr << b << " ";
            std::cerr << std::endl;

            return false;
        }
    }

    return true;

}
} // namespace cartocrow::layout_guide
