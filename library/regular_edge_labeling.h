#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <cartocrow/core/core.h>
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
    int oldWeight;
    int weight;
    // edges of the vertex in COUNTERCLOCKWISE order (combinatorial / cyclic)
    vector<int> edges;

    int left_segment = -1;
    int right_segment = -1;
    int bottom_segment = -1;
    int top_segment = -1;

    double preferred_aspect_ratio = 1.0;
    double preferred_width = 0.0;
    double preferred_height = 0.0;

    int horizontal_order_index = -1;
    int vertical_order_index = -1;

    cartocrow::Color color{255, 255, 255};
};

class RegularEdgeLabeling {
public:
    RegularEdgeLabeling() = default;
    ~RegularEdgeLabeling() = default;

    void buildFromJson(const json &j, bool useSquareAspectRatios);

    bool isValidREL() const;

    void adjustToBB();

    const vector<Vertex> &getVertices()  const { return m_vertices; }
    void updateVertexWeight(int id, int weight) { m_vertices[id].weight = weight; }
    void normalizeVertexWeights();
    void computePreferredSizes();

    std::pair<double, std::vector<int>> getLongestHorizontalPath() const;
    std::pair<double, std::vector<int>> getLongestVerticalPath() const;

    bool mergeMaxHorizontalSegmentFromLeft(int edgeId);
    bool mergeMaxHorizontalSegmentFromRight(int edgeId);
    bool mergeMaxVerticalSegmentFromBottom(int edgeId);
    bool mergeMaxVerticalSegmentFromTop(int edgeId);

    bool mergeLeftMostRedEdge(int edgeId);
    bool mergeRightMostRedEdge(int edgeId);
    bool mergeLowestBlueEdge(int edgeId);
    bool mergeHighestBlueEdge(int edgeId);

    const vector<HalfEdge> &getHalfEdges() const { return m_halfEdges; }

    string otherLabelOfHalfEdge(int h) const;

    // bb box
    void setBoundingBox(const BoundingBox &bb) { m_boundingBox = bb; adjustToBB(); }
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
    int getLastOutgoingBlue(int vertexId) const;
    int getLastIncomingBlue(int vertexId) const;
    int getLastOutgoingRed(int vertexId) const;
    int getLastIncomingRed(int vertexId) const;

    int canonicalHalfEdge(int he) const;

    bool flipEdgeColor(int edgeId);
    bool flipEdgeDiagonally(int edgeId, bool clockwise);
    void revertEdgeDirection(int edgeId);
    void debugCheckAfterFlip(int edgeId) const;

private:
    vector<Vertex> m_vertices;
    unordered_map<string, int> m_labelToIndex;
    vector<HalfEdge> m_halfEdges;

    vector<Vertex> m_initVertices;
    vector<HalfEdge> m_initHalfEdges;

    optional<BoundingBox> m_boundingBox;

    static string dirKey(const std::string &a, const std::string &b);
    static string undirKey(const std::string &a, const std::string &b);

    int findFirstEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const;
    int findLastEdgeOfType(int vertexId, EdgeColor edge_color, bool outgoing) const;

    vector<cartocrow::Color> m_vertColors{ {166, 205, 226}, {255, 255 , 153}, {252, 190, 110}, {250, 153, 153}, {201, 177, 213}, {177, 222, 137}};

};