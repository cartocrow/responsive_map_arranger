#include "rectangular_cartogram_gui.h"

#include <QApplication>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>

#include <nlohmann/json.hpp>

#include "library/regular_edge_labeling.h"

using json = nlohmann::json;


void RectangularCartogramDemo::loadRELData(const std::filesystem::path& dataPath) {
	std::cout << "loading data from " << dataPath << std::endl;

	auto ext = dataPath.extension();

	if (ext != ".json") {
		std::cerr << "Cannot load data from file type " << ext << std::endl;
		return;
	}
	std::ifstream f(dataPath);
	m_RELData = json::parse(f);
	processData();

	m_cartogramType = static_cast<CartogramType>(m_cartogramTypeComboBox->currentData().toInt());

	if (m_cartogramType == RECTANGULAR_CARTOGRAM) {
		m_rectangularDual = std::make_shared<RectangularDual>(m_relPtr);

		m_rectangularDual->setFromREL();

		RectangularCartogramPainting::Options rectCartogramOptions;
		m_rectPainting = std::make_shared<RectangularCartogramPainting>(m_rectangularDual, m_relPtr, rectCartogramOptions);
		m_renderer->addPainting(m_rectPainting, "RectangularCartogram");

		m_demers = nullptr;
	}
	else if (m_cartogramType == DEMERS_CARTOGRAM) {
		m_demers = std::make_shared<DemersCartogram>();
		m_demers->setFromREL(*m_relPtr);

		m_demersPainting = std::make_shared<DemersPainting>(m_demers);
		m_renderer->addPainting(m_demersPainting, "Demer's Cartogram");

		m_rectangularDual = nullptr;
	}

	// REL RENDERING
	RELPainting::Options relDrawingOptions;
	relDrawingOptions.drawLabels = true;
	relDrawingOptions.drawREL = m_showREL->isChecked();

	m_relPainting = std::make_shared<RELPainting>(m_relPtr, m_rectangularDual, m_demers);

	m_renderer->addPainting(m_relPainting, "REL");
}

void RectangularCartogramDemo::loadWeightData(const std::filesystem::path &dataPath) {
	std::cout << "loading data from " << dataPath << std::endl;

	auto ext = dataPath.extension();

	if (ext != ".json") {
		std::cerr << "Cannot load data from file type " << ext << std::endl;
		return;
	}
	std::ifstream f(dataPath);
	m_weightData = json::parse(f);

	if (m_relPtr) {
		m_relPtr->setDataValuesFromJson(m_weightData);
		setCartogramFromREL();
	}
}

void RectangularCartogramDemo::processData() {

    std::cout << "processing data" << std::endl;

    try {
        m_rel.buildFromJson(m_RELData, m_useSquareAspectRatios->checkState()); // will validate & throw if errors found
    } catch (const std::exception& e) {
        std::cerr << "Failed to load REL: " << e.what() << std::endl;
    }

    m_relPtr = std::make_shared<RegularEdgeLabeling>(m_rel);
    m_relPtr->setBoundingBox(BoundingBox{0, 1920 , 0, 1080 });

    std::cout << "====== REL VALIDITY CHECK ======" << std::endl;
    m_relPtr->isValidREL(true);

	if (!m_weightData.is_null()) {
		m_relPtr->setDataValuesFromJson(m_weightData);
	}

}

void RectangularCartogramDemo::setCartogramFromREL() const {
	if (!m_relPtr || !m_relPtr->isValidREL()) return;

	if (m_rectangularDual) {
		m_rectangularDual->setFromREL();
	}
	else if (m_demers) {
		m_demers->setFromREL(*m_relPtr);
	}
}

RectangularCartogramDemo::RectangularCartogramDemo() {
	setWindowTitle("RectangularCartogramDemo");

	m_renderer = new GeometryWidget();
	m_renderer->setDrawAxes(false);
	setCentralWidget(m_renderer);

	auto* dockWidget = new QDockWidget(this);
	addDockWidget(Qt::RightDockWidgetArea, dockWidget);
	auto* vWidget = new QWidget();
	auto* vLayout = new QVBoxLayout(vWidget);
	vLayout->setAlignment(Qt::AlignTop);
	dockWidget->setWidget(vWidget);

	auto* inputSettings = new QLabel("<h3>Input</h3>", vWidget);
	auto loadRELButton = new QPushButton("Load REL (json)");
	auto loadWeightsButton = new QPushButton("Load weights (json)");
	vLayout->addWidget(inputSettings);
	vLayout->addWidget(loadRELButton);
	vLayout->addWidget(loadWeightsButton);

	auto* generalSettings = new QLabel("<h3>General Settings</h3>", vWidget);
	m_cartogramTypeComboBox = new QComboBox(vWidget);
	m_cartogramTypeComboBox->addItem("RectangularCartogram", CartogramType::RECTANGULAR_CARTOGRAM);
	m_cartogramTypeComboBox->addItem("DemersCartogram", CartogramType::DEMERS_CARTOGRAM);
	m_cartogramTypeComboBox->setCurrentIndex(1);
	m_mergeHeuristicComboBox = new QComboBox(vWidget);
	m_mergeHeuristicComboBox->addItem("low-edge-count", LOWEST_EDGE_COUNT);
	m_mergeHeuristicComboBox->addItem("high-seg-low-dir-count", HIGHEST_SEGMENT_LOWEST_DIR_COUNT);
	m_mergeHeuristicComboBox->setCurrentIndex(0);

	m_useSquareAspectRatios = new QCheckBox("Use Square Aspect Ratios", vWidget);
	m_useSquareAspectRatios->setChecked(true);
	vLayout->addWidget(generalSettings);
	vLayout->addWidget(m_cartogramTypeComboBox);
	vLayout->addWidget(m_mergeHeuristicComboBox);
	vLayout->addWidget(m_useSquareAspectRatios);


	auto* debugSettings = new QLabel("<h3>Debug settings</h3>", vWidget);
	m_showREL = new QCheckBox("Show REL");
	m_showREL->setChecked(true);
	m_showLinearOrders = new QCheckBox("Show Linear Orders");
	m_showLinearOrders->setChecked(false);
	vLayout->addWidget(debugSettings);
	vLayout->addWidget(m_showREL);
	vLayout->addWidget(m_showLinearOrders);

	// EDGE SELECTION/MANIPULATION BUTTONS
	auto* selectionLabel = new QLabel("<h3>Selection Actions</h3>", vWidget);
	auto* btnFlipColor = new QPushButton("Flip Color");
	auto* btnFlipDiagCW = new QPushButton("Flip Diagonal ▶ (CW)");
	auto* btnFlipDiagCCW = new QPushButton("Flip Diagonal ◀ (CCW)");
	auto* btnMergeLeftmostEdge = new QPushButton("Merge Leftmost Edge");
	auto* btnMergeRightMostEdge = new QPushButton("Merge Rightmost Edge");
	auto* btnMergeSegmentFromLeft = new QPushButton("Merge Segment (from left)");
	auto* btnMergeSegmentFromRight = new QPushButton("Merge Segment (from right)");
	auto* btnClearSelection = new QPushButton("Clear Selection");

	vLayout->addWidget(selectionLabel);
	vLayout->addWidget(btnFlipColor);
	vLayout->addWidget(btnFlipDiagCW);
	vLayout->addWidget(btnFlipDiagCCW);
	vLayout->addWidget(btnMergeLeftmostEdge);
	vLayout->addWidget(btnMergeRightMostEdge);
	vLayout->addWidget(btnMergeSegmentFromLeft);
	vLayout->addWidget(btnMergeSegmentFromRight);
	vLayout->addWidget(btnClearSelection);


	connect(loadRELButton, &QPushButton::clicked, [this, loadRELButton]() {
		QString startDir = QString::fromStdString(m_settings.getString("dir", "data"));
		std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select region data file"), startDir).toStdString();
		if (filePath.empty()) return;

		m_settings.setString("dir", filePath.parent_path().string());

		loadRELData(filePath);
		loadRELButton->setText(QString::fromStdString(filePath.filename().string()));
		});

	connect(loadWeightsButton, &QPushButton::clicked, [this, loadWeightsButton]() {
		QString startDir = QString::fromStdString(m_settings.getString("dir", "data"));
		std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select region data file"), startDir).toStdString();
		if (filePath.empty()) return;

		loadWeightData(filePath);
		loadWeightsButton->setText(QString::fromStdString(filePath.filename().string()));
	});

	connect(m_showREL, &QCheckBox::toggled, [this]() {
		if (!m_relPainting) return;

		m_relPainting->drawRel(m_showREL->isChecked());
		m_renderer->update();
	});

	connect(m_showLinearOrders, &QCheckBox::toggled, [this]() {
		if (!m_rectPainting) return;

		m_rectPainting->drawLinearOrders(m_showLinearOrders->isChecked());
		m_renderer->update();
	});

	connect(m_cartogramTypeComboBox, qOverload<int>(&QComboBox::currentIndexChanged), [this](int index)	{
		m_cartogramType = static_cast<CartogramType>(m_cartogramTypeComboBox->itemData(index).toInt());

		if (!m_relPtr) return;

		m_renderer->clear();
		m_rectPainting.reset();
		m_demersPainting.reset();
		m_rectangularDual.reset();
		m_demers.reset();

		if (m_cartogramType == RECTANGULAR_CARTOGRAM) {
			m_rectangularDual = std::make_shared<RectangularDual>(m_relPtr);

			m_rectangularDual->setFromREL();

			RectangularCartogramPainting::Options rectCartogramOptions;
			m_rectPainting = std::make_shared<RectangularCartogramPainting>(m_rectangularDual, m_relPtr, rectCartogramOptions);
			m_renderer->addPainting(m_rectPainting, "RectangularCartogram");

		}
		else if (m_cartogramType == DEMERS_CARTOGRAM) {
			m_demers = std::make_shared<DemersCartogram>();
			m_demers->setFromREL(*m_relPtr);

			m_demersPainting = std::make_shared<DemersPainting>(m_demers);
			m_renderer->addPainting(m_demersPainting, "Demer's Cartogram");

		}

		m_relPainting.reset();

		// REL RENDERING
		RELPainting::Options relDrawingOptions;
		relDrawingOptions.drawLabels = true;
		relDrawingOptions.drawREL = m_showREL->isChecked();

		m_relPainting = std::make_shared<RELPainting>(m_relPtr, m_rectangularDual, m_demers);

		m_renderer->addPainting(m_relPainting, "REL");

		m_renderer->update();
	});

	connect(m_cartogramTypeComboBox, qOverload<int>(&QComboBox::currentIndexChanged), [this](int index) {
		m_mergeHeuristic = static_cast<MergeHeuristic>(m_cartogramTypeComboBox->itemData(index).toInt());

		if (!m_relPtr) return;

		m_relPtr->setMergeHeuristic(m_mergeHeuristic);
		m_relPtr->adjustToBB();

		setCartogramFromREL();
		m_renderer->update();
	});

	connect(btnClearSelection, &QPushButton::clicked, [this]() {
		if (m_relPainting) {
			m_relPainting->clearSelection();
			m_renderer->update();
		}
		});

	connect(btnFlipColor, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		for (int he : sels) {
			bool ok = m_relPtr->flipEdgeColor(he);
			if (!ok) std::cerr << "flipEdgeColor failed for halfedge " << he << "\n";
		}
		// after mutating REL, rebuild dual & segment geometry:
		setCartogramFromREL();

		m_renderer->update();
		});

	connect(btnFlipDiagCW, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		for (int he : sels) {
			int heCan = m_relPtr->canonicalHalfEdge(he);
			bool ok = m_relPtr->flipEdgeDiagonally(heCan, /*clockwise=*/true);

			//bool ok = m_relPtr->flipEdgeDiagonally(heCan, /*clockwise=*/true);
			if (!ok) std::cerr << "flipEdgeDiagonally(cw) failed for halfedge " << he << "\n";
		}
		setCartogramFromREL();
		m_renderer->update();
		});

	connect(btnFlipDiagCCW, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		for (int he : sels) {
			int heCan = m_relPtr->canonicalHalfEdge(he);
			bool ok = m_relPtr->flipEdgeDiagonally(heCan, /*clockwise=*/false);
			//bool ok = m_relPtr->flipEdgeDiagonally(heCan, /*clockwise=*/false);
			if (!ok) std::cerr << "flipEdgeDiagonally(ccw) failed for halfedge " << he << "\n";
		}
		setCartogramFromREL();
		m_renderer->update();
		});

	connect(btnMergeLeftmostEdge, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		if (sels.size() > 1) {
			std::cerr << "Can only merge one edge at the time. " << std::endl;
			return;
		}
		for (int he : sels) {

			if (m_relPtr->getHalfEdges()[he].color == RED) {
				m_relPtr->mergeLeftMostRedEdge(he);
			}
			else if (m_relPtr->getHalfEdges()[he].color == BLUE) {
				m_relPtr->mergeLowestBlueEdge(he);
			}
			m_relPainting->clearSelection();
		}

		setCartogramFromREL();

		m_renderer->update();
		});

	connect(btnMergeRightMostEdge, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		if (sels.size() > 1) {
			std::cerr << "Can only merge one edge at the time. " << std::endl;
			return;
		}
		for (int he : sels) {

			if (m_relPtr->getHalfEdges()[he].color == RED) {
				m_relPtr->mergeRightMostRedEdge(he);
			}
			else if (m_relPtr->getHalfEdges()[he].color == BLUE) {
				m_relPtr->mergeHighestBlueEdge(he);
			}
			m_relPainting->clearSelection();
		}

		setCartogramFromREL();

		m_renderer->update();
		});

	connect(btnMergeSegmentFromLeft, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		if (sels.size() > 1) {
			std::cerr << "Can only merge one edge at the time. " << std::endl;
			return;
		}
		for (int he : sels) {
			if (m_relPtr->getHalfEdges()[he].color == RED) {
				m_relPtr->mergeMaxHorizontalSegmentFromLeft(he);
			}
			else if (m_relPtr->getHalfEdges()[he].color == BLUE) {
				m_relPtr->mergeMaxVerticalSegmentFromBottom(he);
			}
			m_relPainting->clearSelection();
		}

		setCartogramFromREL();
		m_renderer->update();
		});

	connect(btnMergeSegmentFromRight, &QPushButton::clicked, [this]() {
		if (!m_relPtr || !m_relPainting) return;
		const auto sels = m_relPainting->getSelectedHalfEdges();
		if (sels.empty()) return;
		if (sels.size() > 1) {
			std::cerr << "Can only merge one edge at the time. " << std::endl;
			return;
		}
		for (int he : sels) {
			if (m_relPtr->getHalfEdges()[he].color == RED) {
				m_relPtr->mergeMaxHorizontalSegmentFromRight(he);
			}
			else if (m_relPtr->getHalfEdges()[he].color == BLUE) {
				m_relPtr->mergeMaxVerticalSegmentFromTop(he);
			}
			m_relPainting->clearSelection();
		}

		setCartogramFromREL();
		m_renderer->update();
		});

	connect(m_renderer, &GeometryWidget::clicked, [this](Point<Inexact> pt) {
		if (!m_relPainting) return;
		float wx = static_cast<float>(pt.x());
		float wy = static_cast<float>(pt.y());
		int he = m_relPainting->pickAndToggleHalfEdgeNear(wx, wy, 8.0 /*tolerance*/);
		if (he >= 0) {
			// optional: print / debug
			std::cout << "Toggled selection halfedge " << he << "\n";
			m_renderer->update();
		}
		});

	// Start bounding box dragging
	connect(m_renderer, &GeometryWidget::dragStarted, this, [this](Point<Inexact> pt) {
		if (!m_relPtr) return;

		auto optbb = m_relPtr->getBoundingBox();
		if (!optbb) return;
		const auto& bb = *optbb;

		double hx = bb.right;
		double hy = bb.top;

		double wx = static_cast<double>(pt.x());
		double wy = static_cast<double>(pt.y());
		double dx = wx - hx;
		double dy = wy - hy;
		double d2 = dx * dx + dy * dy;

		// std::cout << "checking distance for click" << std::endl;
		//
		// std::cout << std::fixed << std::setprecision(6);
		// std::cout << "checking distance for click\n";
		// std::cout << " bb.right,bb.top = (" << hx << ", " << hy << ")\n";
		// std::cout << " pt.x(),pt.y()   = (" << wx << ", " << wy << ")\n";
		// std::cout << " dx,dy = (" << dx << "," << dy << ") d2=" << d2 << "\n";
		// std::cout << " handle tolerance (squared) = " << (m_bboxHandleTolerance*m_bboxHandleTolerance) << "\n";
		if (d2 <= m_bboxHandleTolerance * m_bboxHandleTolerance) {
			m_bboxDragging = true;
			m_dragStartWorld = Point<Inexact>(wx, wy);
			m_bboxBeforeDrag = bb;
		}
		});

	// Update bounding box while dragging
	connect(m_renderer, &GeometryWidget::dragMoved, this, [this](Point<Inexact> pt) {
		if (!m_bboxDragging) return;
		if (!m_relPtr) return;

		//std::cout << "updating bb" << std::endl;

		double newRight = static_cast<double>(pt.x());
		double newTop = static_cast<double>(pt.y());
		double left = m_bboxBeforeDrag.left;
		double bottom = m_bboxBeforeDrag.bottom;

		//clamp to min size
		if (newRight < left + m_bboxMinWidth) newRight = left + m_bboxMinWidth;
		if (newTop < bottom + m_bboxMinHeight) newTop = bottom + m_bboxMinHeight;

		BoundingBox newbb;
		newbb.left = left;
		newbb.right = newRight;
		newbb.bottom = bottom;
		newbb.top = newTop;

		m_relPtr->setBoundingBox(newbb);

		setCartogramFromREL();

		//m_demers->setFromREL(*m_relPtr);

		m_renderer->update();
		});

	connect(m_renderer, &GeometryWidget::dragEnded, this, [this](const Point<Inexact>& pt) {
		if (!m_bboxDragging) return;
		m_bboxDragging = false;
		});
}

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	RectangularCartogramDemo demo;
	demo.show();
	return app.exec();
}