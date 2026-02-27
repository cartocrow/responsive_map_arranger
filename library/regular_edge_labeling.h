#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <nlohmann/json_fwd.hpp>
#include "geometry_types.h"

using json = nlohmann::json;

using namespace std;

enum EdgeColor {
    RED = 0,
    BLUE = 1,
    BLACK = 2
};

struct HalfEdge {
    int32_t vertex = -1; //vertex index
    int32_t twin = -1; // opposite half-edge
    //int32_t next = -1; // next half edge around face
    //int32_t prev = -1; // previous half edge around face
    EdgeColor color = BLACK;
    bool outgoing;
    string id_str;
};

struct Vertex {
    string label;
    int weight;
    // edges of the vertex in COUNTERCLOCKWISE order (combinatorial / cyclic)
    vector<int> edges;

    int left_segment = -1;
    int right_segment = -1;
    int bottom_segment = -1;
    int top_segment = -1;
};

class RegularEdgeLabeling {
public:
    RegularEdgeLabeling() = default;
    ~RegularEdgeLabeling() = default;

    void buildFromJson(const json &j);

    bool isValidREL() const;

    const vector<Vertex> &getVertices()  const { return m_vertices; }
    void updateVertexWeight(int id, int weight) { m_vertices[id].weight = weight; }
    const vector<HalfEdge> &getHalfEdges() const { return m_halfEdges; }

    string otherLabelOfHalfEdge(int h) const;

    // bb box
    void setBoundingBox(const BoundingBox &bb) { m_boundingBox = bb; }
    optional<BoundingBox> getBoundingBox() const { return m_boundingBox; }
    bool hasBoundingBox() const noexcept { return static_cast<bool>(m_boundingBox); }
    void clearBoundingBox() { m_boundingBox.reset(); }

    void printSummary() const;

    void setVertexSegmentIndices(int vertexId, int leftSeg, int rightSeg, int bottomSeg, int topSeg);

    int getVertexLeftSegment(int v) const;
    int getVertexRightSegment(int v) const;
    int getVertexBottomSegment(int v) const;
    int getVertexTopSegment(int v) const;

    int getPreviousCyclicEdge(const int edgeId) const;
    int getNextCyclicEdge(const int edgeId) const;

    int getVertexDegree(const int vertexId) const { return m_vertices[vertexId].edges.size(); }
    int getFirstOutgoingBlue(int vertexId) const;
    int getFirstIncomingBlue(int vertexId) const;
    int getFirstOutgoingRed(int vertexId) const;
    int getFirstIncomingRed(int vertexId) const;
    int getlastOutgoingBlue(int vertexId) const;
    int getlastIncomingBlue(int vertexId) const;
    int getlastOutgoingRed(int vertexId) const;
    int getlastIncomingRed(int vertexId) const;

    int canonicalHalfEdge(int he) const;

    bool flipEdgeColor(const int edgeId);
    bool flipEdgeDiagonally(int edgeId, bool clockwise);
    void debugCheckAfterFlip(int edgeId) const;

private:
    vector<Vertex> m_vertices;
    unordered_map<string, int> m_labelToIndex;
    vector<HalfEdge> m_halfEdges;

    optional<BoundingBox> m_boundingBox;

    static string dirKey(const std::string &a, const std::string &b);
    static string undirKey(const std::string &a, const std::string &b);

    int findFirstEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const;
    int findLastEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const;

};