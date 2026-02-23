// rel_painting.cpp
#include "rel_painting.h"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace cartocrow::rel_vis;

RELPainting::RELPainting(std::shared_ptr<RELmap> relmap,
                         std::shared_ptr<RectangularDual> dual)
    : m_relmap(std::move(relmap)), m_dual(std::move(dual)), m_options() {}

RELPainting::RELPainting(std::shared_ptr<RELmap> relmap,
                         std::shared_ptr<RectangularDual> dual,
                         Options opts)
    : m_relmap(std::move(relmap)), m_dual(std::move(dual)), m_options(std::move(opts)) {}

void RELPainting::paint(Renderer &renderer) const {
    if (!m_relmap) return;
    if (!m_options.drawREL) return;

    const size_t nRegions = m_relmap->size();
    if (nRegions == 0) return;

    // compute positions: either from dual centroids or from fall-back grid
    std::vector<PointI> pos(nRegions);

    bool usingDual = false;
    if (m_dual && m_dual->rectangles().size() == nRegions) {
        usingDual = true;
        for (size_t i = 0; i < nRegions; ++i) {
            const auto &r = m_dual->getRect(static_cast<unsigned int>(i));
            const double cx = 0.5 * (r.left + r.right);
            const double cy = 0.5 * (r.bottom + r.top);
            pos[i] = PointI(cx, cy);
        }
    } else {
        // fall back to grid layout using horizontal / vertical order if available
        std::vector<int> xIdx(nRegions, -1), yIdx(nRegions, -1);

        int x = 0;
        for (auto id : m_relmap->horizontalOrder()) {
            if (id < static_cast<RegionId>(nRegions)) xIdx[id] = x++;
        }
        int y = 0;
        for (auto id : m_relmap->verticalOrder()) {
            if (id < static_cast<RegionId>(nRegions)) yIdx[id] = y++;
        }
        int nextX = x, nextY = y;
        for (size_t id = 0; id < nRegions; ++id) {
            if (xIdx[id] == -1) xIdx[id] = nextX++;
            if (yIdx[id] == -1) yIdx[id] = nextY++;
        }
        for (size_t id = 0; id < nRegions; ++id) {
            const double px = static_cast<double>(xIdx[id]) * static_cast<double>(m_options.horizontalSpacing);
            const double py = static_cast<double>(yIdx[id]) * static_cast<double>(m_options.verticalSpacing);
            pos[id] = PointI(px, py);
        }
    }

    const double arrowSize = static_cast<double>(m_options.arrowSize);
    const double strokeW = static_cast<double>(m_options.strokeWidth);

    // draw red edges (vertical constraints — in your project red means bottom->top)
    for (RegionId u = 0; u < static_cast<RegionId>(nRegions); ++u) {
        const auto &ru = m_relmap->get(u);
        for (auto v : ru.red_out) {
            if (v >= static_cast<RegionId>(nRegions)) continue;
            const auto &rv = m_relmap->get(v);

            // frame edges: both endpoints are frame labels => thick black undirected line
            if (isFrameLabel(ru.label) && isFrameLabel(rv.label)) {
                renderer.setMode(Renderer::stroke);
                renderer.setStroke({ m_options.colorEdgeFrame[0],
                                     m_options.colorEdgeFrame[1],
                                     m_options.colorEdgeFrame[2] },
                                    std::max(2.0, strokeW * 2.0));
                renderer.draw(Segment<Inexact>(pos[u], pos[v]));
                continue;
            }

            // normal red directed edge
            renderer.setMode(Renderer::stroke);
            renderer.setStroke({ m_options.colorRedEdge[0],
                                 m_options.colorRedEdge[1],
                                 m_options.colorRedEdge[2] },
                                std::max(1.0, strokeW));
            renderer.draw(Segment<Inexact>(pos[u], pos[v]));

            if (m_options.drawEdgeArrowheads) {
                // compute direction unit vector
                const Inexact::Vector_2 dir = pos[v] - pos[u];
                const double len = std::sqrt(static_cast<double>(dir.squared_length()));
                if (len > 1e-9) {
                    const double ux = static_cast<double>(dir.x()) / len;
                    const double uy = static_cast<double>(dir.y()) / len;

                    // tip stops a bit before centroid v
                    const double offset = arrowSize * 0.6;
                    const double tipx = static_cast<double>(pos[v].x()) - ux * offset;
                    const double tipy = static_cast<double>(pos[v].y()) - uy * offset;
                    const PointI tip(tipx, tipy);

                    const double perpx = -uy;
                    const double perpy = ux;

                    const double half = arrowSize * 0.5;
                    const double p1x = tipx - ux * arrowSize + perpx * half;
                    const double p1y = tipy - uy * arrowSize + perpy * half;
                    const double p2x = tipx - ux * arrowSize - perpx * half;
                    const double p2y = tipy - uy * arrowSize - perpy * half;

                    PointI p1(p1x, p1y);
                    PointI p2(p2x, p2y);

                    cartocrow::Polygon<Inexact> arrow;
                    arrow.push_back(tip);
                    arrow.push_back(p1);
                    arrow.push_back(p2);

                    renderer.setFill({ m_options.colorRedEdge[0],
                                       m_options.colorRedEdge[1],
                                       m_options.colorRedEdge[2] });
                    renderer.setMode(Renderer::fill | Renderer::stroke);
                    renderer.draw(arrow);
                }
            }
        }
    }

    // draw blue edges (horizontal constraints — blue means left->right in your setup)
    for (RegionId u = 0; u < static_cast<RegionId>(nRegions); ++u) {
        const auto &ru = m_relmap->get(u);
        for (auto v : ru.blue_out) {
            if (v >= static_cast<RegionId>(nRegions)) continue;
            const auto &rv = m_relmap->get(v);

            // frame edges: both endpoints are frame labels => thick black undirected line
            if (isFrameLabel(ru.label) && isFrameLabel(rv.label)) {
                renderer.setMode(Renderer::stroke);
                renderer.setStroke({ m_options.colorEdgeFrame[0],
                                     m_options.colorEdgeFrame[1],
                                     m_options.colorEdgeFrame[2] },
                                    std::max(2.0, strokeW * 2.0));
                renderer.draw(Segment<Inexact>(pos[u], pos[v]));
                continue;
            }

            // normal blue directed edge
            renderer.setMode(Renderer::stroke);
            renderer.setStroke({ m_options.colorBlueEdge[0],
                                 m_options.colorBlueEdge[1],
                                 m_options.colorBlueEdge[2] },
                                std::max(1.0, strokeW));
            renderer.draw(Segment<Inexact>(pos[u], pos[v]));

            if (m_options.drawEdgeArrowheads) {
                const Inexact::Vector_2 dir = pos[v] - pos[u];
                const double len = std::sqrt(static_cast<double>(dir.squared_length()));
                if (len > 1e-9) {
                    const double ux = static_cast<double>(dir.x()) / len;
                    const double uy = static_cast<double>(dir.y()) / len;

                    const double offset = arrowSize * 0.6;
                    const double tipx = static_cast<double>(pos[v].x()) - ux * offset;
                    const double tipy = static_cast<double>(pos[v].y()) - uy * offset;
                    const PointI tip(tipx, tipy);

                    const double perpx = -uy;
                    const double perpy = ux;

                    const double half = arrowSize * 0.5;
                    const double p1x = tipx - ux * arrowSize + perpx * half;
                    const double p1y = tipy - uy * arrowSize + perpy * half;
                    const double p2x = tipx - ux * arrowSize - perpx * half;
                    const double p2y = tipy - uy * arrowSize - perpy * half;

                    PointI p1(p1x, p1y);
                    PointI p2(p2x, p2y);

                    cartocrow::Polygon<Inexact> arrow;
                    arrow.push_back(tip);
                    arrow.push_back(p1);
                    arrow.push_back(p2);

                    renderer.setFill({ m_options.colorBlueEdge[0],
                                       m_options.colorBlueEdge[1],
                                       m_options.colorBlueEdge[2] });
                    renderer.setMode(Renderer::fill | Renderer::stroke);
                    renderer.draw(arrow);
                }
            }
        }
    }

    // draw labels at centroids if requested (nodes themselves are not drawn)
    if (m_options.drawLabels) {
        for (RegionId u = 0; u < static_cast<RegionId>(nRegions); ++u) {
            const std::string &label = m_relmap->get(u).label;
            const PointI &p = pos[u];
            renderer.setFill({ m_options.colorText[0],
                               m_options.colorText[1],
                               m_options.colorText[2] });
            // Use your renderer's text API. Typical: renderer.drawText(label, p);
            renderer.drawText(p, label);
        }
    }
}