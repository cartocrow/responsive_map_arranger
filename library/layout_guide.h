#pragma once

#include <cartocrow/core/core.h>

using namespace std;

namespace cartocrow::layout_guide {
enum EdgeLabel {
    VERTICAL = 0,
    HORIZONTAL = 1,
    BLACK = 2
};

struct LayoutElement {
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
};

struct HalfEdge {
    int m_vertex = -1; // incident vertex
    int m_twin = -1; // twin half edge

    int m_outgoing;
    EdgeLabel m_edgeLabel;
};


class LayoutGuide {
public:
    LayoutGuide(vector<LayoutElement> layoutElements, vector<HalfEdge> halfEdges);

private:
    // contains the map elements. First four vertices represent the outer Vertices West, North, East and South
    vector<LayoutElement> m_elements;
    // contains all half edges incident to layout elements of m_elements
    vector<HalfEdge> m_halfEdges;

    bool isValidLayoutElement(int const &v) const {
        return 0 <= v && v < static_cast<int>(m_elements.size());
    }
    bool isValidHalfEdge(int const &he) const {
        return 0 <= he && he < static_cast<int>(m_halfEdges.size());
    }
    int getCanonicalHalfEdge(int const &he) const;
};
} // namespace cartocrow::layout_guide
