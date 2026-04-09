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

#include <cartocrow/core/core.h>

#include "regular_edge_labeling.h"

using namespace std;

namespace cartocrow::layout_guide {
enum EdgeLabel {
    VERTICAL = 0,
    HORIZONTAL = 1,
    BLACK = 2
};

enum EdgeType {
    OUTGOING_VERTICAL = 0,
    OUTGOING_HORIZONTAL = 1,
    INCOMING_VERTICAL = 2,
    INCOMING_HORIZONTAL = 3,
    NONE = 4
};

struct Vertex {
    string m_label;
    double m_relativeArea;
    double m_aspectRatio;
    double m_area;
    double m_width;
    double m_height;

    bool m_outerVertex = false;
    bool m_seaRegion = false;

    int m_horizontal_order_index = -1;
    int m_vertical_order_index = -1;

    // half edges of the vertex in COUNTERCLOCKWISE cyclic order
    vector<int> m_edges;

    int degree() const { return m_edges.size(); }
};

struct HalfEdge {
    int m_vertex = -1; // incident vertex
    int m_twin = -1; // twin half edge

    bool m_outgoing;
    EdgeLabel m_edgeLabel;

    EdgeType edgeType() const {
        if (m_edgeLabel == VERTICAL && m_outgoing) return OUTGOING_VERTICAL;
        if (m_edgeLabel == VERTICAL && !m_outgoing) return INCOMING_VERTICAL;
        if (m_edgeLabel == HORIZONTAL && m_outgoing) return OUTGOING_HORIZONTAL;
        if (m_edgeLabel == HORIZONTAL && !m_outgoing) return INCOMING_HORIZONTAL;
        return NONE;
    }
};


class LayoutGuide {
public:
    LayoutGuide(vector<Vertex> vertices, vector<HalfEdge> halfEdges);

    //bool isValidREL(bool debugging = false) const;

    const vector<Vertex> &getVertices() const {return m_vertices; }
    const vector<HalfEdge> &getHalfEdges() const { return m_halfEdges; }
    void setVertexWeight(const int vId, const double weight) { m_vertices[vId].m_relativeArea = weight; };

    int getCanonicalHalfEdge(int const &heId) const;
    int getNextCyclicEdge(int const &heId) const;
    int getPreviousCyclicEdge(int const &heId) const;
    int getCyclicPositionOfHalfEdge(int const &heId) const;

    int getFirstEdgeOfType(int vId, EdgeLabel edgeLabel, bool outgoing) const;
    int getLastEdgeOfType(int vId, EdgeLabel edgeLabel, bool outgoing) const;
    int getFirstOutgoingHorizontal(int const vId) const { return getFirstEdgeOfType(vId, HORIZONTAL, true); }
    int getFirstIncomingHorizontal(int const vId) const { return getFirstEdgeOfType(vId, HORIZONTAL, false); }
    int getLastOutgoingHorizontal(int const vId) const { return getLastEdgeOfType(vId, HORIZONTAL, true); }
    int getLastIncomingHorizontal(int const vId) const { return getLastEdgeOfType(vId, HORIZONTAL, false); }
    int getFirstOutgoingVertical(int const vId) const { return getFirstEdgeOfType(vId, VERTICAL, true); }
    int getFirstIncomingVertical(int const vId) const { return getFirstEdgeOfType(vId, VERTICAL, false); }
    int getLastOutgoingVertical(int const vId) const { return getLastEdgeOfType(vId, VERTICAL, true); }
    int getLastIncomingVertical(int const vId) const { return getLastEdgeOfType(vId, VERTICAL, false); }

    bool flipEdgeColor(int const &heID);
    bool flipEdgeDiagonally(int const &heId, bool clockwise);
    bool redirectEdge(int const &heID);

private:
    // contains the vertices. First four vertices represent the outer vertices West, North, East and South
    vector<Vertex> m_vertices;
    // contains all half edges incident to layout elements of m_elements
    vector<HalfEdge> m_halfEdges;

    bool isValidVertex(int const &v) const {
        return 0 <= v && v < static_cast<int>(m_vertices.size());
    }
    bool isValidHalfEdge(int const &he) const {
        return 0 <= he && he < static_cast<int>(m_halfEdges.size());
    }

    bool checkRelHalfEdgesConsistency() const;
    bool checkCyclicEdgeTypeOrder() const;

};
} // namespace cartocrow::layout_guide
