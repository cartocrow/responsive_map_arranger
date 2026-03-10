// rel_painting.cpp
#include "rel_painting.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <array>
#include <limits>

// If RegularEdgeLabeling and the nested types are in a namespace (e.g. rel::),
// either add `using namespace rel;` here, or qualify them below (e.g. rel::RegularEdgeLabeling).
// The code below assumes the class is visible as `RegularEdgeLabeling` and the types
// `RegularEdgeLabeling::HalfEdge` and `RegularEdgeLabeling::Vertex` exist.

RELPainting::RELPainting(std::shared_ptr<RegularEdgeLabeling> rel,
                         std::shared_ptr<RectangularDual> dual)
    : m_rel(std::move(rel)), m_dual(std::move(dual)), m_options() {}

RELPainting::RELPainting(std::shared_ptr<RegularEdgeLabeling> rel,
                         std::shared_ptr<RectangularDual> dual,
                         Options opts)
    : m_rel(std::move(rel)), m_dual(std::move(dual)), m_options(std::move(opts)) {}

void RELPainting::setRegularEdgeLabeling(std::shared_ptr<RegularEdgeLabeling> rel) {
    m_rel = std::move(rel);
}


void RELPainting::paint(Renderer &renderer) const {
    if (!m_rel) return;
    if (!m_options.drawREL) return;

    const auto &vertices = m_rel->getVertices();
    const size_t nRegions = vertices.size();
    if (nRegions == 0) return;

    // ---------- positions: ALWAYS use RectangularDual (user requested) ----------
    if (!m_dual) {
        std::cerr << "RELPainting::paint: expected RectangularDual but m_dual is null\n";
        return;
    }
    if (m_dual->rectangles().size() != nRegions) {
        std::cerr << "RELPainting::paint: RectangularDual size (" << m_dual->rectangles().size()
                  << ") does not match number of REL vertices (" << nRegions << ")\n";
        return;
    }

    // compute centroids as PointI-compatible doubles then construct PointI when calling renderer
    std::vector<std::pair<double,double>> pos(nRegions);
    for (size_t i = 0; i < nRegions; ++i) {
        const auto &r = m_dual->getRect(static_cast<unsigned int>(i));
        const double cx = 0.5 * (r.left + r.right);
        const double cy = 0.5 * (r.bottom + r.top);
        pos[i] = { cx, cy };
    }

    const double arrowSize = static_cast<double>(m_options.arrowSize);
    const double strokeW = static_cast<double>(m_options.strokeWidth);

    // convenience lambdas for drawing with your renderer API
    auto draw_segment = [&](double ax, double ay, double bx, double by,
                            const std::array<double,3> &strokeColor, double width) {
        renderer.setMode(Renderer::stroke);
        renderer.setStroke({ static_cast<int>(strokeColor[0]), static_cast<int>(strokeColor[1]), static_cast<int>(strokeColor[2]) }, width);
        PointI A(ax, ay);
        PointI B(bx, by);
        renderer.draw(cartocrow::Segment<Inexact>(A, B));
    };

    auto draw_arrow_triangle = [&](double tipx, double tipy,
                                   double p1x, double p1y,
                                   double p2x, double p2y,
                                   const std::array<double,3> &fillColor) {
        PolygonI arrow;
        arrow.push_back(PointI(tipx, tipy));
        arrow.push_back(PointI(p1x, p1y));
        arrow.push_back(PointI(p2x, p2y));
        renderer.setFill({ static_cast<int>(fillColor[0]), static_cast<int>(fillColor[1]), static_cast<int>(fillColor[2]) });
        renderer.setMode(Renderer::fill | Renderer::stroke);
        renderer.draw(arrow);
    };

    // iterate half-edges and draw explicit outgoing ones (one per directed edge)
    const auto &halfedges = m_rel->getHalfEdges();
    for (int hi = 0; hi < (int)halfedges.size(); ++hi) {
        const auto &h = halfedges[hi];
        if (!h.outgoing) continue;             // draw only outgoing halfedges
        //if (!h.is_explicit) continue;          // draw only explicit edges (from input outgoing lists)

        const int u = h.vertex;
        const int twin = h.twin;
        if (u < 0 || u >= (int)nRegions) continue;
        if (twin < 0 || twin >= (int)halfedges.size()) continue;
        const int v = halfedges[twin].vertex;
        if (v < 0 || v >= (int)nRegions) continue;

        const double ax = pos[u].first;
        const double ay = pos[u].second;
        const double bx = pos[v].first;
        const double by = pos[v].second;

        // frame edges: both endpoints are frame labels => thick black undirected line
        const std::string &alabel = vertices[u].label;
        const std::string &blabel = vertices[v].label;
        if (isFrameLabel(alabel) && isFrameLabel(blabel)) {
            continue;
            draw_segment(ax, ay, bx, by,
                         { m_options.colorEdgeFrame[0],
                           m_options.colorEdgeFrame[1],
                           m_options.colorEdgeFrame[2] },
                         std::max(2.0, strokeW * 2.0));
            continue;
        }

        // choose color & draw
        if (h.color == RED) {
            draw_segment(ax, ay, bx, by,
                         { m_options.colorRedEdge[0],
                           m_options.colorRedEdge[1],
                           m_options.colorRedEdge[2] },
                         std::max(1.0, strokeW));

            if (m_options.drawEdgeArrowheads) {
                double dx = bx - ax;
                double dy = by - ay;
                double len = std::sqrt(dx*dx + dy*dy);
                if (len > 1e-9) {
                    double ux = dx / len;
                    double uy = dy / len;
                    const double offset = arrowSize * 0.6;
                    const double tipx = bx - ux * offset;
                    const double tipy = by - uy * offset;
                    const double perpx = -uy;
                    const double perpy = ux;
                    const double half = arrowSize * 0.5;
                    const double p1x = tipx - ux * arrowSize + perpx * half;
                    const double p1y = tipy - uy * arrowSize + perpy * half;
                    const double p2x = tipx - ux * arrowSize - perpx * half;
                    const double p2y = tipy - uy * arrowSize - perpy * half;

                    draw_arrow_triangle(tipx, tipy, p1x, p1y, p2x, p2y,
                                        { m_options.colorRedEdge[0],
                                          m_options.colorRedEdge[1],
                                          m_options.colorRedEdge[2] });
                }
            }
        } else if (h.color == BLUE) {
            draw_segment(ax, ay, bx, by,
                         { m_options.colorBlueEdge[0],
                           m_options.colorBlueEdge[1],
                           m_options.colorBlueEdge[2] },
                         std::max(1.0, strokeW));

            if (m_options.drawEdgeArrowheads) {
                double dx = bx - ax;
                double dy = by - ay;
                double len = std::sqrt(dx*dx + dy*dy);
                if (len > 1e-9) {
                    double ux = dx / len;
                    double uy = dy / len;
                    const double offset = arrowSize * 0.6;
                    const double tipx = bx - ux * offset;
                    const double tipy = by - uy * offset;
                    const double perpx = -uy;
                    const double perpy = ux;
                    const double half = arrowSize * 0.5;
                    const double p1x = tipx - ux * arrowSize + perpx * half;
                    const double p1y = tipy - uy * arrowSize + perpy * half;
                    const double p2x = tipx - ux * arrowSize - perpx * half;
                    const double p2y = tipy - uy * arrowSize - perpy * half;

                    draw_arrow_triangle(tipx, tipy, p1x, p1y, p2x, p2y,
                                        { m_options.colorBlueEdge[0],
                                          m_options.colorBlueEdge[1],
                                          m_options.colorBlueEdge[2] });
                }
            }
        } else {
            // BLACK or other
            draw_segment(ax, ay, bx, by,
                         { m_options.colorEdgeFrame[0],
                           m_options.colorEdgeFrame[1],
                           m_options.colorEdgeFrame[2] },
                         std::max(1.0, strokeW));
        }

        if (m_selectedHalfEdges.count(hi)) {
            // use a bright highlight color and thicker stroke
            draw_segment(ax, ay, bx, by,
                         { m_options.colorSelection[0], m_options.colorSelection[1], m_options.colorSelection[2] },
                         std::max(2.0, strokeW * 2.5));
        }
    } // end halfedge loop

    // draw labels
    if (m_options.drawLabels) {
        for (size_t i = 0; i < nRegions; ++i) {
            const auto &label = vertices[i].label;
            const auto &pc = pos[i];
            PointI p(pc.first, pc.second);
            renderer.setFill({ static_cast<int>(m_options.colorText[0]),
                               static_cast<int>(m_options.colorText[1]),
                               static_cast<int>(m_options.colorText[2]) });
            renderer.drawText(p, label);
        }
    }
}

// Helper: squared distance from point P to segment AB
static double pointSegmentDist2(double px, double py, double ax, double ay, double bx, double by) {
    double vx = bx - ax;
    double vy = by - ay;
    double wx = px - ax;
    double wy = py - ay;
    double c = vx*vx + vy*vy;
    if (c <= 1e-12) {
        // degenerate segment
        double dx = px - ax, dy = py - ay;
        return dx*dx + dy*dy;
    }
    double t = (wx*vx + wy*vy) / c;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double qx = ax + vx * t;
    double qy = ay + vy * t;
    double dx = px - qx;
    double dy = py - qy;
    return dx*dx + dy*dy;
}

int RELPainting::pickHalfEdgeNear(double wx, double wy, double tol) const {
    if (!m_rel || !m_dual) return -1;
    const auto &vertices = m_rel->getVertices();
    const size_t nRegions = vertices.size();
    if (nRegions == 0) return -1;
    if (m_dual->rectangles().size() != nRegions) return -1;

    // compute centroids (same as paint)
    std::vector<std::pair<double,double>> pos(nRegions);
    for (size_t i = 0; i < nRegions; ++i) {
        const auto &r = m_dual->getRect(static_cast<unsigned int>(i));
        pos[i] = { 0.5 * (r.left + r.right), 0.5 * (r.bottom + r.top) };
    }

    const auto &halfedges = m_rel->getHalfEdges();
    int bestHe = -1;
    double bestD2 = tol * tol;
    for (int hi = 0; hi < (int)halfedges.size(); ++hi) {
        const auto &h = halfedges[hi];
        if (!h.outgoing) continue;
        int u = h.vertex;
        int twin = h.twin;
        if (u < 0 || u >= (int)nRegions) continue;
        if (twin < 0 || twin >= (int)halfedges.size()) continue;
        int v = halfedges[twin].vertex;
        if (v < 0 || v >= (int)nRegions) continue;

        double ax = pos[u].first, ay = pos[u].second;
        double bx = pos[v].first, by = pos[v].second;

        double d2 = pointSegmentDist2(wx, wy, ax, ay, bx, by);
        if (d2 <= bestD2) {
            bestD2 = d2;
            bestHe = hi;
        }
    }
    return bestHe;
}

int RELPainting::pickAndToggleHalfEdgeNear(double wx, double wy, double tol) {
    int he = pickHalfEdgeNear(wx, wy, tol);
    if (he < 0) return -1;
    if (m_selectedHalfEdges.count(he)) m_selectedHalfEdges.erase(he);
    else m_selectedHalfEdges.insert(he);
    return he;
}

void RELPainting::clearSelection() {
    m_selectedHalfEdges.clear();
}

void RELPainting::selectHalfEdge(int halfedge) {
    if (halfedge >= 0) m_selectedHalfEdges.insert(halfedge);
}

void RELPainting::deselectHalfEdge(int halfedge) {
    m_selectedHalfEdges.erase(halfedge);
}