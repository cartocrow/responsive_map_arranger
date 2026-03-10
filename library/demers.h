#pragma once

#include <cartocrow/core/core.h>
#include <cartocrow/renderer/geometry_painting.h>
#include <cartocrow/renderer/geometry_renderer.h>

#include "regular_edge_labeling.h"

class DemersPainting;
class DemersPosition;

class DemersCartogram {
public:
	DemersCartogram() = default;

	void setFromREL(RegularEdgeLabeling& rel);
private:

	using Rect = cartocrow::Rectangle<cartocrow::Inexact>;

	Rect box;
	std::vector<DemersPosition> locations;

	friend class DemersPainting;
};

struct DemersPosition {
	using Pt = cartocrow::Point<cartocrow::Inexact>;
	using Rect = cartocrow::Rectangle<cartocrow::Inexact>;

	const cartocrow::Color color;
	const std::string label;
	const Pt center;
	const Rect rectangle;

	DemersPosition(const Vertex& v, double x, double y, double rad) : color(v.color), label(v.label), center(x, y), rectangle(x - rad, y - rad, x + rad, y + rad) {}
};

class DemersPainting : public cartocrow::renderer::GeometryPainting {
public:
	using Renderer = cartocrow::renderer::GeometryRenderer;

	DemersPainting(std::shared_ptr<DemersCartogram> cartogram) : m_cartogram(cartogram) {};

	void paint(Renderer& renderer) const override;

private:
	std::shared_ptr<DemersCartogram> m_cartogram;
};