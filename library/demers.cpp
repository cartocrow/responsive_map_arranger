#include "demers.h"

#include <glpk.h>

#include "geometry_types.h"


void DemersCartogram::setFromREL(RegularEdgeLabeling& rel) {

	assert(rel.hasBoundingBox());

	//const auto weight_to_rad = [](int weight) { return weight; };
	const auto radius_of = [](const Vertex& v) { return v.isLandRegion ? std::sqrt(v.weight) / 2.0 : 0; };

	// build LP
	glp_prob* lp = glp_create_prob();
	glp_set_obj_dir(lp, GLP_MIN);

	// given n interior vertices, the program has 2n + 1 variables
	// 0: scale
	// and two for each vertex (x and y)
	// plus a bunch of helper variables...

	const int var_scale = glp_add_cols(lp, 1);
	glp_set_col_bnds(lp, var_scale, GLP_LO, 0, 1);

	BoundingBox bb = rel.getBoundingBox().value();
	box = Rect(bb.left, bb.bottom, bb.right, bb.top);

	const vector<Vertex>& vs = rel.getVertices();
	const int var_vertex_start = glp_add_cols(lp, 2 * (vs.size() - 4));

	double scale_factor = 1;

	for (int i = 4; i < vs.size(); i++) { // skip first four (bounding vertices)
		const Vertex& v = vs[i];

		double rad = radius_of(v);

		int C = glp_add_rows(lp, 4);

		// stay in box horizontally
		const int xv = var_vertex_start + 2 * (i - 4);
		glp_set_col_bnds(lp, xv, GLP_FR, 0, 0);
		glp_set_obj_coef(lp, xv, 0.0000001);

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
		glp_set_obj_coef(lp, yv, 0.0000001);

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

			auto from_rad = radius_of(vs[from]);
			auto to_rad = radius_of(vs[to]);
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

			scale_factor += bb.width() + bb.height();
		}
	}

	// minimization objective, we minimize -scale to maximize scale
	glp_set_obj_coef(lp, var_scale, -scale_factor);

	// solve LP
	glp_smcp param;
	glp_init_smcp(&param);
	param.msg_lev = GLP_MSG_ERR;

	int lazy = 0;
	int rounds = 0;
	bool changed = true;
	while (changed) {
		rounds++;
		if (0 != glp_simplex(lp, &param)) {
			std::cout << "Unsolvable LP?" << std::endl;
			break;
		}

		// construct locations

		double scale = glp_get_col_prim(lp, var_scale);

		locations.clear();
		for (int i = 4; i < vs.size(); i++) { // skip first four (bounding vertices)
			const Vertex& v = vs[i];

			double x = glp_get_col_prim(lp, var_vertex_start + 2 * (i - 4));
			double y = glp_get_col_prim(lp, var_vertex_start + 2 * (i - 4) + 1);

			double scaled_rad = scale * radius_of(v);

			locations.push_back(DemersPosition(v, x, y, scaled_rad));
		}

		const int n = locations.size();
		changed = false;
		for (int i = 0; i < n; i++) {
			const DemersPosition a = locations[i];
			const double a_rad = (a.rectangle.xmax() - a.rectangle.xmin()) / 2.0;
			for (int j = i + 1; j < n; j++) {
				const DemersPosition b = locations[j];
				const double b_rad = (b.rectangle.xmax() - b.rectangle.xmin()) / 2.0;
				const double scaled_dist = a_rad + b_rad - 0.000001;

				const double dx = std::abs(a.center.x() - b.center.x());
				const double dy = std::abs(a.center.y() - b.center.y());

				if (dx < scaled_dist && dy < scaled_dist) {
					// overlap

					auto from_rad = radius_of(vs[i+4]);
					auto to_rad = radius_of(vs[j+4]);
					auto dist = from_rad + to_rad;

					if (dx <= dy) {
						// overlap is strongest horizontally, so separate vertically

						const int C = glp_add_rows(lp, 1);

						int from_y, to_y;
						if (a.center.y() <= b.center.y()) {
							from_y = var_vertex_start + 2 * (i)+1;
							to_y = var_vertex_start + 2 * (j)+1;
						}
						else {
							from_y = var_vertex_start + 2 * (j)+1;
							to_y = var_vertex_start + 2 * (i)+1;
						}
						// to.y - from.y >= scale * dist 
						const int vars[] = { 0, to_y, from_y, var_scale };
						const double facs[] = { 0,  1, -1, -dist };
						glp_set_row_bnds(lp, C, GLP_LO, 0, 0);
						glp_set_mat_row(lp, C, 3, vars, facs);

						lazy++;
						changed = true;
					}
					else {
						// overlap is strongest vertically, so separate horizontally

						const int C = glp_add_rows(lp, 1);

						int from_x, to_x;
						if (a.center.x() <= b.center.x()) {
							from_x = var_vertex_start + 2 * (i);
							to_x = var_vertex_start + 2 * (j);
						}
						else {
							from_x = var_vertex_start + 2 * (j);
							to_x = var_vertex_start + 2 * (i);
						}
						// to.x - from.x >= scale * dist
						const int vars[] = { 0, to_x, from_x, var_scale };
						const double facs[] = { 0,  1, -1, -dist };
						glp_set_row_bnds(lp, C, GLP_LO, 0, 0);
						glp_set_mat_row(lp, C, 3, vars, facs);

						lazy++;
						changed = true;
					}
				}
			}
		}
	}

	std::cout << "Solved in " << rounds << " rounds; " << lazy << " lazy constraints added." << std::endl;
 
	glp_delete_prob(lp);
}

void DemersPainting::paint(Renderer& renderer) const {

	if (!m_cartogram) {
		return;
	}

	renderer.setMode(Renderer::fill | Renderer::stroke);

	auto relVertices = m_REL->getVertices();
	for (size_t i = 0; i < m_cartogram->locations.size(); i++) {
		if (!relVertices[i +4].isLandRegion) continue;
		DemersPosition dp = m_cartogram->locations[i];

		renderer.setFill(relVertices[i+4].color);
		renderer.draw(dp.rectangle);

		if (m_drawLabels) {
			renderer.setFill({ 0,0,0 });
			renderer.drawText(dp.center, dp.label);
		}
	}

	// draw boundingbox
	renderer.setStroke({ 102,102,102 }, 2);
	renderer.setMode(Renderer::stroke);
	renderer.draw(m_cartogram->box);
};