#pragma once

#include <cartocrow/core/core.h>
#include "regular_edge_labeling.h"

class DemersCartogram {
public:
	using Pt = cartocrow::Point<cartocrow::Inexact>;

	DemersCartogram() = default;

	void setFromREL(RegularEdgeLabeling& rel);
private:

	std::vector<Pt> locations;
};