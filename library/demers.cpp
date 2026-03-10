#include "demers.h"

#include <glpk.h>

#include "geometry_types.h"


void DemersCartogram::setFromREL(RegularEdgeLabeling& rel) {
	locations.clear();

	assert(rel.hasBoundingBox());

	//const auto weight_to_rad = [](int weight) { return weight; };
	const auto weight_to_rad = [](int weight) { return std::sqrt(weight) / 2.0; };

	// build LP
	glp_prob* lp = glp_create_prob();

	// given n interior vertices, the program has 2n + 1 variables
	// 0: scale
	// and two for each vertex (x and y)
	// plus a bunch of helper variables...

	const int var_scale = glp_add_cols(lp, 1);
	glp_set_col_bnds(lp, var_scale, GLP_LO, 0, 1);

	// minimization objective, we minimize -scale to maximize scale
	glp_set_obj_dir(lp, GLP_MIN);
	glp_set_obj_coef(lp, var_scale, -1000);

	BoundingBox bb = rel.getBoundingBox().value();
	box = Rect(bb.left, bb.bottom, bb.right, bb.top);

	const vector<Vertex>& vs = rel.getVertices();
	const int var_vertex_start = glp_add_cols(lp, 2 * (vs.size() - 4));

	for (int i = 4; i < vs.size(); i++) { // skip first four (bounding vertices)
		Vertex v = vs[i];

		double rad = weight_to_rad(v.weight);

		int C = glp_add_rows(lp, 4);

		// stay in box horizontally
		const int xv = var_vertex_start + 2 * (i - 4);
		glp_set_col_bnds(lp, xv, GLP_FR, 0, 0);

		{ // v.x + scale * rad <= bb.right 
			const int vars[] = { 0, xv, var_scale };
			const double facs[] = { 0,  1.0, rad };
			glp_set_row_bnds(lp, C, GLP_UP, 0, bb.right);
			glp_set_mat_row(lp, C, 2, vars, facs);
			C++;
		}

		{ // v.x - scale * rad >= bb.left  
			const int vars[] = { 0, xv, var_scale };
			const double facs[] = { 0,  1.0, -rad };
			glp_set_row_bnds(lp, C, GLP_LO, bb.left, 0);
			glp_set_mat_row(lp, C, 2, vars, facs);
			C++;
		}

		// stay in box vertically
		const int yv = xv + 1;
		glp_set_col_bnds(lp, yv, GLP_FR, 0, 0);
		{ // v.y + scale * rad <= bb.top 
			const int vars[] = { 0, yv, var_scale };
			const double facs[] = { 0,  1.0, rad };
			glp_set_row_bnds(lp, C, GLP_UP, 0, bb.top);
			glp_set_mat_row(lp, C, 2, vars, facs);
			C++;
		}

		{ // v.x - scale * rad >= bb.bottom  
			const int vars[] = { 0, yv, var_scale };
			const double facs[] = { 0,  1.0, -rad };
			glp_set_row_bnds(lp, C, GLP_LO, bb.bottom, 0);
			glp_set_mat_row(lp, C, 2, vars, facs);
			C++;
		}
	}

	const vector<HalfEdge>& es = rel.getHalfEdges();
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

			int from_x = var_vertex_start + 2 * (from - 4);
			int from_y = from_x + 1;
			int to_x = var_vertex_start + 2 * (to - 4);
			int to_y = to_x + 1;

			int C = glp_add_rows(lp, 7);
			if (he.color == BLUE) {
				// separate horizontally
				// to.x - from.x >= scale * dist
				const int vars[] = { 0, to_x, from_x, var_scale };
				const double facs[] = { 0,  1, -1, -dist };
				glp_set_row_bnds(lp, C, GLP_LO, 0, 0);
				glp_set_mat_row(lp, C, 3, vars, facs);
				C++;
			}
			else {
				// separate vertically
				// to.y - from.y >= scale * dist 
				const int vars[] = { 0, to_y, from_y, var_scale };
				const double facs[] = { 0,  1, -1, -dist };
				glp_set_row_bnds(lp, C, GLP_LO, 0, 0);
				glp_set_mat_row(lp, C, 3, vars, facs);
				C++;
			}

			const int h = glp_add_cols(lp, 2);

			glp_set_col_bnds(lp, h, GLP_LO, 0, 0); // h >= 0
			{ // h >= fromx - tox - dist
				const int vars[] = { 0, h, to_x, from_x };
				const double facs[] = { 0,  1, 1, -1 };
				glp_set_row_bnds(lp, C, GLP_LO, -dist, 0);
				glp_set_mat_row(lp, C, 3, vars, facs);
				C++;
			}
			{ // h >= tox - fromx - dist
				const int vars[] = { 0, h, to_x, from_x };
				const double facs[] = { 0,  1, -1, 1 };
				glp_set_row_bnds(lp, C, GLP_LO, -dist, 0);
				glp_set_mat_row(lp, C, 3, vars, facs);
				C++;
			}

			const int v = h + 1;

			glp_set_col_bnds(lp, v, GLP_LO, 0, 0); // h >= 0
			{ // v >= fromy - toy - dist
				const int vars[] = { 0, v, to_y, from_y };
				const double facs[] = { 0,  1, 1, -1 };
				glp_set_row_bnds(lp, C, GLP_LO, -dist, 0);
				glp_set_mat_row(lp, C, 3, vars, facs);
				C++;
			}
			{ // v >= toy - fromy - dist
				const int vars[] = { 0, v, to_y, from_y };
				const double facs[] = { 0,  1, -1, 1 };
				glp_set_row_bnds(lp, C, GLP_LO, -dist, 0);
				glp_set_mat_row(lp, C, 3, vars, facs);
				C++;
			}

			glp_set_obj_coef(lp, h, 1);
			glp_set_obj_coef(lp, v, 1);
		}
	}


	// solve LP
	if (0 != glp_simplex(lp, NULL)) {
		std::cout << "Unsolvable LP?" << std::endl;
		return;
	}

	// construct locations

	double scale = glp_get_col_prim(lp, var_scale);

	for (int i = 4; i < vs.size(); i++) { // skip first four (bounding vertices)
		const Vertex v = vs[i];

		double x = glp_get_col_prim(lp, var_vertex_start + 2 * (i - 4));
		double y = glp_get_col_prim(lp, var_vertex_start + 2 * (i - 4) + 1);

		double scaled_rad = CGAL::to_double(scale * weight_to_rad(v.weight));

		locations.push_back(DemersPosition(v, x, y, scaled_rad));
	}

	glp_delete_prob(lp);
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