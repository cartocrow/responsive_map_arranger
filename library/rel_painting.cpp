// rel_painting.cpp
#include "rel_painting.h"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <optional>

using namespace cartocrow::rel_vis;

// helper: is this one of the outer frame labels?
static bool isFrameLabel(const std::string &s) {
    return s == "North" || s == "South" || s == "West" || s == "East";
}

// find a region id by exact label, return optional RegionId
static std::optional<RegionId> findRegionIdByLabel(const RELmap &m, const std::string &label) {
    const size_t n = m.size();
    for (RegionId i = 0; i < static_cast<RegionId>(n); ++i) {
        if (m.get(i).label == label) return i;
    }
    return std::nullopt;
}

RELPainting::RELPainting(std::shared_ptr<RELmap> relmap, Options &&opts)
    : m_relmap(std::move(relmap)), m_options(std::move(opts)) { }

cartocrow::Polygon<Inexact> RELPainting::makeCirclePolygon(const PointI &p, Number<Inexact> r, int n) const {
    Polygon<Inexact> poly;
    poly.reserve(n);
    const double rd = static_cast<double>(r);
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * M_PI * double(i) / double(n);
        const double x = static_cast<double>(p.x()) + rd * std::cos(a);
        const double y = static_cast<double>(p.y()) + rd * std::sin(a);
        poly.push_back(PointI(x, y));
    }
    return poly;
}

void RELPainting::paint(Renderer &renderer) const {
    if (!m_relmap) return;
    if (!options().drawREL) return;

    // 1) number of regions
    const size_t nRegions = m_relmap->size();
    if (nRegions == 0) return;

    // 2) build x/y indices from orders (if present)
    std::vector<int> xIdx(nRegions, -1), yIdx(nRegions, -1);

    int x = 0;
    for (auto id : m_relmap->horizontalOrder()) {
        if (id < static_cast<RegionId>(nRegions)) xIdx[id] = x++;
    }
    int y = 0;
    for (auto id : m_relmap->verticalOrder()) {
        if (id < static_cast<RegionId>(nRegions)) yIdx[id] = y++;
    }

    // Assign remaining indices sequentially
    int nextX = x, nextY = y;
    for (size_t id = 0; id < nRegions; ++id) {
        if (xIdx[id] == -1) xIdx[id] = nextX++;
        if (yIdx[id] == -1) yIdx[id] = nextY++;
    }

    // compute current minima/maxima
    const int minX = *std::min_element(xIdx.begin(), xIdx.end());
    const int maxX = *std::max_element(xIdx.begin(), xIdx.end());
    const int minY = *std::min_element(yIdx.begin(), yIdx.end());
    const int maxY = *std::max_element(yIdx.begin(), yIdx.end());

    // mid positions (integer)
    const int midX = (minX + maxX) / 2;
    const int midY = (minY + maxY) / 2;

    // helper to find occupant of a coordinate (xv,yv)
    auto findOccupant = [&](int xv, int yv) -> std::optional<RegionId> {
        for (RegionId i = 0; i < static_cast<RegionId>(nRegions); ++i) {
            if (xIdx[i] == xv && yIdx[i] == yv) return i;
        }
        return std::nullopt;
    };

    // helper to swap coordinates between two ids
    auto swapCoords = [&](RegionId a, RegionId b) {
        std::swap(xIdx[a], xIdx[b]);
        std::swap(yIdx[a], yIdx[b]);
    };

    // For each frame node, compute target (tx,ty). If occupant exists, swap; else set.
    // West -> (minX, midY)
    if (auto w = findRegionIdByLabel(*m_relmap, "West"); w) {
        RegionId rid = *w;
        const int tx = minX, ty = midY;
        if (!(xIdx[rid] == tx && yIdx[rid] == ty)) {
            if (auto occ = findOccupant(tx, ty)) swapCoords(rid, *occ);
            else { xIdx[rid] = tx; yIdx[rid] = ty; }
        }
    }

    // East -> (maxX, midY)
    if (auto e = findRegionIdByLabel(*m_relmap, "East"); e) {
        RegionId rid = *e;
        const int tx = maxX, ty = midY;
        if (!(xIdx[rid] == tx && yIdx[rid] == ty)) {
            if (auto occ = findOccupant(tx, ty)) swapCoords(rid, *occ);
            else { xIdx[rid] = tx; yIdx[rid] = ty; }
        }
    }

    // South -> (midX, minY)
    if (auto s = findRegionIdByLabel(*m_relmap, "South"); s) {
        RegionId rid = *s;
        const int tx = midX, ty = minY;
        if (!(xIdx[rid] == tx && yIdx[rid] == ty)) {
            if (auto occ = findOccupant(tx, ty)) swapCoords(rid, *occ);
            else { xIdx[rid] = tx; yIdx[rid] = ty; }
        }
    }

    // North -> (midX, maxY)
    if (auto n = findRegionIdByLabel(*m_relmap, "North"); n) {
        RegionId rid = *n;
        const int tx = midX, ty = maxY;
        if (!(xIdx[rid] == tx && yIdx[rid] == ty)) {
            if (auto occ = findOccupant(tx, ty)) swapCoords(rid, *occ);
            else { xIdx[rid] = tx; yIdx[rid] = ty; }
        }
    }

    // 3) compute positions (world coordinates)
    std::vector<PointI> pos(nRegions);
    for (size_t id = 0; id < nRegions; ++id) {
        const double px = static_cast<double>(xIdx[id]) * static_cast<double>(m_options.horizontalSpacing);
        const double py = static_cast<double>(yIdx[id]) * static_cast<double>(m_options.verticalSpacing);
        pos[id] = PointI(px, py);
    }

    // convenience numbers
    const double nodeRadius = static_cast<double>(m_options.nodeRadius);
    const double arrowSize  = static_cast<double>(m_options.arrowSize);

    // 4) draw red edges (frame edges overridden to black undirected)
    for (RegionId u = 0; u < static_cast<RegionId>(nRegions); ++u) {
        const auto &ru = m_relmap->get(u);
        for (auto v : ru.red_out) {
            if (v >= static_cast<RegionId>(nRegions)) continue;
            const auto &rv = m_relmap->get(v);

            // frame edges: both endpoints are in {N,S,W,E} => draw thick black undirected line
            if (isFrameLabel(ru.label) && isFrameLabel(rv.label)) {
                renderer.setMode(Renderer::stroke);
                renderer.setStroke({0,0,0}, std::max(2.5, nodeRadius / 3.0));
                renderer.draw(Segment<Inexact>(pos[u], pos[v]));
                continue;
            }

            // normal red directed edge with optional arrow
            renderer.setMode(Renderer::stroke);
            renderer.setStroke({220,0,0}, std::max(1.0, nodeRadius / 5.0));
            renderer.draw(Segment<Inexact>(pos[u], pos[v]));

            if (m_options.drawEdgeArrowheads) {
                // compute direction unit vector
                const Inexact::Vector_2 dir = pos[v] - pos[u];
                const double len = std::sqrt(static_cast<double>(dir.squared_length()));
                if (len > 1e-9) {
                    const double ux = static_cast<double>(dir.x()) / len;
                    const double uy = static_cast<double>(dir.y()) / len;

                    // tip slightly before node v so arrow doesn't overlap circle
                    const double offset = nodeRadius + arrowSize * 0.3;
                    const double tipx = static_cast<double>(pos[v].x()) - ux * offset;
                    const double tipy = static_cast<double>(pos[v].y()) - uy * offset;
                    const PointI tip(tipx, tipy);

                    // perpendicular
                    const double perpx = -uy;
                    const double perpy = ux;

                    // base points
                    const double p1x = tipx - ux * arrowSize + perpx * (arrowSize * 0.5);
                    const double p1y = tipy - uy * arrowSize + perpy * (arrowSize * 0.5);
                    const double p2x = tipx - ux * arrowSize - perpx * (arrowSize * 0.5);
                    const double p2y = tipy - uy * arrowSize - perpy * (arrowSize * 0.5);

                    PointI p1(p1x, p1y);
                    PointI p2(p2x, p2y);

                    Polygon<Inexact> arrow;
                    arrow.push_back(tip);
                    arrow.push_back(p1);
                    arrow.push_back(p2);

                    renderer.setFill({220,0,0});
                    renderer.setMode(Renderer::fill | Renderer::stroke);
                    renderer.draw(arrow);
                }
            }
        }
    }

    // 5) draw blue edges (frame edges again treated as black undirected)
    for (RegionId u = 0; u < static_cast<RegionId>(nRegions); ++u) {
        const auto &ru = m_relmap->get(u);
        for (auto v : ru.blue_out) {
            if (v >= static_cast<RegionId>(nRegions)) continue;
            const auto &rv = m_relmap->get(v);

            // frame edge
            if (isFrameLabel(ru.label) && isFrameLabel(rv.label)) {
                renderer.setMode(Renderer::stroke);
                renderer.setStroke({0,0,0}, std::max(2.5, nodeRadius / 3.0));
                renderer.draw(Segment<Inexact>(pos[u], pos[v]));
                continue;
            }

            // normal blue directed edge with optional arrow
            renderer.setMode(Renderer::stroke);
            renderer.setStroke({0,100,220}, std::max(1.0, nodeRadius / 5.0));
            renderer.draw(Segment<Inexact>(pos[u], pos[v]));

            if (m_options.drawEdgeArrowheads) {
                const Inexact::Vector_2 dir = pos[v] - pos[u];
                const double len = std::sqrt(static_cast<double>(dir.squared_length()));
                if (len > 1e-9) {
                    const double ux = static_cast<double>(dir.x()) / len;
                    const double uy = static_cast<double>(dir.y()) / len;

                    const double offset = nodeRadius + arrowSize * 0.3;
                    const double tipx = static_cast<double>(pos[v].x()) - ux * offset;
                    const double tipy = static_cast<double>(pos[v].y()) - uy * offset;
                    const PointI tip(tipx, tipy);

                    const double perpx = -uy;
                    const double perpy = ux;

                    const double p1x = tipx - ux * arrowSize + perpx * (arrowSize * 0.5);
                    const double p1y = tipy - uy * arrowSize + perpy * (arrowSize * 0.5);
                    const double p2x = tipx - ux * arrowSize - perpx * (arrowSize * 0.5);
                    const double p2y = tipy - uy * arrowSize - perpy * (arrowSize * 0.5);

                    PointI p1(p1x, p1y);
                    PointI p2(p2x, p2y);

                    Polygon<Inexact> arrow;
                    arrow.push_back(tip);
                    arrow.push_back(p1);
                    arrow.push_back(p2);

                    renderer.setFill({0,100,220});
                    renderer.setMode(Renderer::fill | Renderer::stroke);
                    renderer.draw(arrow);
                }
            }
        }
    }

    // 6) draw nodes on top
    for (RegionId u = 0; u < static_cast<RegionId>(nRegions); ++u) {
        const PointI &p = pos[u];
        if (m_options.drawNodeFill) {
            renderer.setFill({ m_options.colorNodeFill[0],
                               m_options.colorNodeFill[1],
                               m_options.colorNodeFill[2] });
        }
        if (m_options.drawNodeStroke) {
            renderer.setStroke({ m_options.colorNodeStroke[0],
                                 m_options.colorNodeStroke[1],
                                 m_options.colorNodeStroke[2] },
                                std::max(1.0, nodeRadius / 6.0));
        }
        renderer.setMode(Renderer::fill | Renderer::stroke);
        renderer.draw(makeCirclePolygon(p, m_options.nodeRadius));

        if (m_options.drawLabels) {
            const std::string &label = m_relmap->get(u).label;
            // If your renderer supports text, replace the cross below with, e.g.:
            // renderer.drawText(label, p);
            // Otherwise we draw a small cross (guaranteed to compile)
            const double w = nodeRadius * 0.6;
            renderer.setMode(Renderer::stroke);
            renderer.setStroke({ m_options.colorNodeStroke[0],
                                 m_options.colorNodeStroke[1],
                                 m_options.colorNodeStroke[2] },
                                std::max(1.0, nodeRadius / 8.0));
            renderer.draw(Segment<Inexact>(PointI(p.x() - w, p.y() - w), PointI(p.x() + w, p.y() + w)));
            renderer.draw(Segment<Inexact>(PointI(p.x() - w, p.y() + w), PointI(p.x() + w, p.y() - w)));
        }
    }
}