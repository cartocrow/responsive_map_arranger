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

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <cartocrow/core/core.h>
#include <nlohmann/json_fwd.hpp>

using json = nlohmann::json;
using namespace std;

namespace cartocrow::layout_guide {
enum OuterVertices {
    WEST = 0,
    NORTH = 1,
    EAST = 2,
    SOUTH = 3,
};

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
    string label;
    double relativeArea;
    double aspectRatio;
    double area;
    double width;
    double height;

    bool outerVertex = false;
    bool seaRegion = false;

    int horizontal_order_index = -1;
    int vertical_order_index = -1;

    // half edges of the vertex in COUNTERCLOCKWISE cyclic order
    vector<int> edges;

    int degree() const { return edges.size(); }
};

struct HalfEdge {
    int vertex = -1; // incident vertex
    int twin = -1; // twin half edge

    bool outgoing;
    EdgeLabel edgeLabel;

    EdgeType edgeType() const {
        if (edgeLabel == VERTICAL && outgoing) return OUTGOING_VERTICAL;
        if (edgeLabel == VERTICAL && !outgoing) return INCOMING_VERTICAL;
        if (edgeLabel == HORIZONTAL && outgoing) return OUTGOING_HORIZONTAL;
        if (edgeLabel == HORIZONTAL && !outgoing) return INCOMING_HORIZONTAL;
        return NONE;
    }
};


class LayoutGuide {
public:
    LayoutGuide(vector<Vertex> vertices, vector<HalfEdge> halfEdges);
    LayoutGuide(const json &j);

    bool isValidREL(bool debugging = false) const;

    const vector<Vertex> &getVertices() const {return m_vertices; }
    const vector<HalfEdge> &getHalfEdges() const { return m_halfEdges; }
    void scaleRelativeVertexSizes(const int vId, const double weight) const;

    std::pair<double, std::vector<int>> getLongestPath(EdgeLabel edgeLabel, int source, int sink, const function<double(int)> &vertexCost, int minNodes = 1) const;

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
    // maps element labels to vertex indices
    unordered_map<string, int> m_labelToIndex;


    bool isValidVertex(int const &v) const {
        return 0 <= v && v < static_cast<int>(m_vertices.size());
    }
    bool isValidHalfEdge(int const &he) const {
        return 0 <= he && he < static_cast<int>(m_halfEdges.size());
    }

    vector<vector<int>> buildAdjacencyMatrix(EdgeLabel edgeLabel) const;
    optional<vector<int>> getTopologicalOrder(EdgeLabel edgeLabel) const;
    optional<vector<int>> getTopologicalOrder(EdgeLabel edgeLabel, const vector<vector<int>> &adj) const;

    void computeRelativeVertexWeights();

    bool checkRelHalfEdgesConsistency() const;
    bool checkCyclicEdgeTypeOrder() const;
    bool isEdgeTypeAcyclic(EdgeLabel edgeLabel) const;

};
} // namespace cartocrow::layout_guide
