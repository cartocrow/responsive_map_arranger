// rectangular_cartogram_painting.cpp
#include "rectangular_cartogram_painting.h"

#include <sstream>
#include <iomanip>

#include "regular_edge_labeling.h"


RectangularCartogramPainting::RectangularCartogramPainting(std::shared_ptr<RectangularDual> dual,
                                                           std::shared_ptr<RegularEdgeLabeling> relmap)
    : RectangularCartogramPainting(std::move(dual), std::move(relmap), Options{}) { }

RectangularCartogramPainting::RectangularCartogramPainting(std::shared_ptr<RectangularDual> dual,
                                                           std::shared_ptr<RegularEdgeLabeling> relmap,
                                                           Options opts)
    : m_dual(std::move(dual)), m_relmap(std::move(relmap)), m_options(std::move(opts))
{ }

void RectangularCartogramPainting::paint(Renderer &renderer) const {
    if (!m_dual) return;

    const std::size_t n = m_dual->size();
    if (n == 0) return;

    // draw rectangles
    for (std::size_t id = 4; id < n; ++id) {
        const auto &r = m_dual->getRect(static_cast<unsigned int>(id));
        // create polygon in CCW order: (left,bottom) -> (right,bottom) -> (right,top) -> (left,top)
        const PointI p0(r.left,  r.bottom);
        const PointI p1(r.right, r.bottom);
        const PointI p2(r.right, r.top);
        const PointI p3(r.left,  r.top);

        PolygonI rect;
        rect.push_back(p0);
        rect.push_back(p1);
        rect.push_back(p2);
        rect.push_back(p3);

        if (m_options.drawFill) {
            renderer.setFill(r.color);
            //renderer.setFill({ m_options.colorFill[0], m_options.colorFill[1], m_options.colorFill[2] });
        }
        if (m_options.drawStroke) {
            renderer.setStroke({ m_options.colorStroke[0], m_options.colorStroke[1], m_options.colorStroke[2] }, m_options.strokeWidth);
        }
        renderer.setMode(Renderer::fill | Renderer::stroke);
        renderer.draw(rect);

        // draw label in center
        if (m_options.drawLabels) {
            // determine label string: prefer relmap labels if provided, otherwise use numeric id
            std::string label;
            if (m_relmap) {
                // safe bounds check: ensure rel has enough vertices and id is valid
                const auto &verts = m_relmap->getVertices();
                if (id < verts.size()) {
                    label = verts[id].label;
                    if (m_options.drawLinearOrders)
                        label.append("\n [" + std::to_string(verts[id].horizontal_order_index) + ":" + std::to_string(verts[id].vertical_order_index) + "]");
                } else {
                    std::ostringstream ss;
                    ss << "R" << id;
                    label = ss.str();
                }
            } else {
                std::ostringstream ss;
                ss << "R" << id;
                label = ss.str();
            }

            // compute center
            const double cx = 0.5 * (r.left + r.right);
            const double cy = 0.5 * (r.bottom + r.top);
            const PointI center(cx, cy);

            // set text color
            renderer.setFill({ m_options.colorText[0], m_options.colorText[1], m_options.colorText[2] });

            // draw text using renderer's text API
            renderer.drawText(center, label);
        }
    }
}