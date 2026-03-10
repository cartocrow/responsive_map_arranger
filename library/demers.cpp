#include "demers.h"

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

using Program = CGAL::Quadratic_program<ET>;
using Solution = CGAL::Quadratic_program_solution<ET>;

#include "geometry_types.h"


void DemersCartogram::setFromREL(RegularEdgeLabeling& rel) {
	locations.clear();

	assert(rel.hasBoundingBox());

	//const auto weight_to_rad = [](int weight) { return ET(weight); };
	const auto weight_to_rad = [](int weight) { return ET(std::sqrt(weight) / 2.0); };

	// build LP
	Program lp(CGAL::SMALLER, true, 0, false, 0);

	// given n interior vertices, the program has 2n + 1 variables
	// 0: scale
	// and two for each vertex (x and y)
	// plus a bunch of helper variables...

	const int scale_var = 0;

	BoundingBox bb = rel.getBoundingBox().value();
	box = Rect(bb.left, bb.bottom, bb.right, bb.top);

	const vector<Vertex>& vs = rel.getVertices();
	int C = 0;
	for (int i = 4; i < vs.size(); i++) { // skip first four (bounding vertices)
		Vertex v = vs[i];

		auto rad = weight_to_rad(v.weight);

		// stay in box horizontally
		const int xv = 2 * (i - 4) + 1;
		lp.set_a(xv, C, 1); lp.set_a(scale_var, C, rad); lp.set_b(C, bb.top); C++; // v.x + scale * rad <= bb.top 
		lp.set_a(xv, C, -1); lp.set_a(scale_var, C, rad); lp.set_b(C, -bb.bottom); C++; // v.x - scale * rad >= bb.bottom  

		// stay in box vertically
		const int yv = xv + 1;
		lp.set_a(yv, C, 1); lp.set_a(scale_var, C, rad); lp.set_b(C, bb.right); C++; // v.y + scale * rad <= bb.right
		lp.set_a(yv, C, -1); lp.set_a(scale_var, C, rad); lp.set_b(C, -bb.left); C++; // v.y - scale * rad >= bb.left
	}

	const vector<HalfEdge>& es = rel.getHalfEdges();
	int he_vars = 2 * (vs.size() - 4) + 2 + 1; // first free variable for edges
	for (HalfEdge he : es) {
		if (he.outgoing) {
			int from = he.vertex;
			int to = es[he.twin].vertex;

			if (from < 4 || to < 4) {
				// to bounding vertices
				continue;
			}

			auto from_rad = weight_to_rad(vs[from].weight);
			auto to_rad = weight_to_rad(vs[to].weight);
			auto dist = from_rad + to_rad;

			int from_x = 2 * (from - 4) + 1;
			int from_y = from_x + 1;
			int to_x = 2 * (to - 4) + 1;
			int to_y = to_x + 1;
			if (he.color == BLUE) { // separate horizontally
				lp.set_a(from_x, C, 1); lp.set_a(to_x, C, -1);  lp.set_a(scale_var, C, dist); lp.set_b(C, 0); C++; // to - from >= scale * dist 
			}
			else { // separate vertically
				lp.set_a(from_y, C, 1); lp.set_a(to_y, C, -1);  lp.set_a(scale_var, C, dist); lp.set_b(C, 0); C++; // to - from >= scale * dist 
			}

			const int h = he_vars++;
			const int v = he_vars++;

			lp.set_a(h, C, -1); lp.set_b(C, 0); C++; // h >= 0
			lp.set_a(v, C, -1); lp.set_b(C, 0); C++; // v >= 0

			lp.set_a(from_x, C, 1); lp.set_a(to_x, C, -1); lp.set_a(h, C, -1); lp.set_b(C, dist); C++; // h >= fromx - tox - dist
			lp.set_a(from_x, C, -1); lp.set_a(to_x, C, 1); lp.set_a(h, C, -1); lp.set_b(C, dist); C++; // h >= tox - fromx - dist

			lp.set_a(from_y, C, 1); lp.set_a(to_y, C, -1); lp.set_a(v, C, -1); lp.set_b(C, dist); C++; // v >= fromy - toy - dist
			lp.set_a(from_y, C, -1); lp.set_a(to_y, C, 1); lp.set_a(v, C, -1); lp.set_b(C, dist); C++; // v >= toy - fromy - dist
		}
	}


	// minimization objective, we minimize -scale to maximize scale
	lp.set_c(scale_var, -1000);


	CGAL::print_linear_program(std::cout, lp, "Demers LP");

	// solve LP
	Solution s = CGAL::solve_linear_program(lp, ET());
	if (!s.is_optimal()) {
		std::cout << "Unsolvable LP?" << std::endl;
		return;
	}

	//assert(s.solves_linear_program(lp));
	//std::cout << s;

	// construct locations

	auto it = s.variable_values_begin();
	auto scale = *it;
	it++;

	int vi = 4;
	while (vi < vs.size()) {
		double x = CGAL::to_double(*it);
		it++;
		double y = CGAL::to_double(*it);

		const Vertex v = vs[vi];
		double scaled_rad = CGAL::to_double(scale * weight_to_rad(v.weight));

		locations.push_back(DemersPosition(v, x, y, scaled_rad));
		it++;
		vi++;
	}
}

void DemersPainting::paint(Renderer& renderer) const {

	if (!m_cartogram) {
		return;
	}

	renderer.setStroke({ 0,0,0 }, 1);
	renderer.setMode(Renderer::stroke);
	renderer.draw(m_cartogram->box);

	renderer.setMode(Renderer::fill | Renderer::stroke);
	for (DemersPosition dp : m_cartogram->locations) {
		renderer.setFill(dp.color);
		renderer.draw(dp.rectangle);

		renderer.setFill({ 0,0,0 });
		renderer.drawText(dp.center, dp.label);
	}
};