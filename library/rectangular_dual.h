// rectangular_dual.h
#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <optional>
#include <unordered_set>

#include "regular_edge_labeling.h"


enum SegmentType {
    SEGMENT_UNKNOWN = 0,
    SEGMENT_HORIZONTAL = 1, // blue
    SEGMENT_VERTICAL = 2    // red
};

struct Segment {
    SegmentType type = SEGMENT_UNKNOWN; // blue or red
    std::vector<int> halfedges;          // half-edge indices belonging to this maximal segment
    std::vector<int> incoming_vertices;  // for horizontal: below rects, for vertical: left rects
    std::vector<int> outgoing_vertices;  // for horizontal: above rects, for vertical: right rects

    bool fixedSegment = false;
    double coord = 0.0;
    double gradientValue = 0.0;
    //coord
    //gradient value
};

class RELmap;

class RegularEdgeLabeling;
enum EdgeColor;

class RectangularDual {
public:
    struct Rect {
        double left;
        double right;
        double bottom;
        double top;

        cartocrow::Color color{255, 255, 255};

        double area() const {
            return (right - left) * (top - bottom);
        }

        double aspectRatio() const {
            return (right - left) / (top - bottom);
        }
    };

    RectangularDual(const shared_ptr<RegularEdgeLabeling> &rel) : m_REL(rel) {};

    void setFromREL();

    bool hasValidSegmentCoords() const;
    double computeAreaDeviation();
    void fixRectangleAreas();
    bool computeMaximalSegments();
    std::vector<Segment> getMaximalSegments() const { return maximalSegments; };
    bool computeSegmentPositions(double cell_size = 1.0);
    bool computeRectanglesFromSegments();


    std::size_t size() const noexcept { return rects.size(); }
    const Rect &getRect(std::uint32_t id) const;
    const std::vector<Rect> &rectangles() const noexcept { return rects; }

    using Rectangle = cartocrow::Rectangle<cartocrow::Inexact>;
    const Rectangle getBox() const noexcept { return box; }

    // STATISTICS
    double aspectRatioDeviation(int vertexId) const;
    double totalAspectRatioDeviation() const;

private:

    // helper functions (implemented in cpp)
    bool topoSort(const std::vector<std::vector<std::uint32_t>> &adj,
                  std::vector<std::uint32_t> &order) const;

    // packHorizontal / packVertical compute integer indices;
    // the semantics here: leftIndex[v] = integer column index for v;
    // bottomIndex[v] = integer row index for v.
    // These are simple longest-path packers; they do not currently produce multi-column spans.
    void packHorizontal(const std::vector<std::vector<std::uint32_t>> &adj,
                        const std::vector<std::uint32_t> &topo,
                        std::vector<int> &leftIndex, int &maxRight) const;

    void packVertical(const std::vector<std::vector<std::uint32_t>> &adj,
                      const std::vector<std::uint32_t> &topo,
                      std::vector<int> &bottomIndex, int &maxTop) const;


    shared_ptr<RegularEdgeLabeling> m_REL;

    Rectangle box;

    std::vector<Segment> maximalSegments;

    // adjacency lists (size = number of rectangles/vertices)
    // horAdj: left -> right edges (blue)
    // verAdj: bottom -> top edges (red)
    std::vector<std::vector<std::uint32_t>> horAdj;
    std::vector<std::vector<std::uint32_t>> verAdj;

    std::vector<Rect> rects;
};