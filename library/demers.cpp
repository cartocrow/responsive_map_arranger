#include "demers.h"

void DemersCartogram::setFromREL(RegularEdgeLabeling& rel) {
	locations.clear();

	for (Vertex v : rel.getVertices()) {
		locations.push_back(Pt(0, 0));
	}
}