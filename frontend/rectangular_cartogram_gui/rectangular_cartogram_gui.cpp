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


void RectangularCartogramDemo::loadData(const std::filesystem::path &dataPath) {
    std::cout << "loading data from " << dataPath << std::endl;

    auto ext = dataPath.extension();

    if (ext != ".json") {
        std::cerr << "Cannot load data from file type " << ext << std::endl;
        return;
    }
    std::ifstream f(dataPath);
    m_projectData = json::parse(f);
    processData();

    // std::cout << m_relPtr->flipEdgeDiagonally(12, true) << std::endl;
    // m_relPtr->debugCheckAfterFlip(12);
    //
    // m_relPtr->flipEdgeColor(12);
    //
    // m_relPtr->printSummary();



    m_rectangularDual = std::make_shared<RectangularDual>();

    // CREATE RECTANGULAR DUAL
    if (!m_rectangularDual->initializeFromREL(m_rel)) {
        std::cerr << "Failed to compute rectangular dual. You might want to check for cycles" << std::endl;
    }


    // RENDERING
    RELPainting::Options relDrawingOptions;
    relDrawingOptions.drawLabels = true;
    relDrawingOptions.drawREL = m_showREL->isChecked();

    RectangularCartogramPainting::Options rectCartogramOptions;

    m_rectPainting = std::make_shared<RectangularCartogramPainting>(m_rectangularDual, m_relPtr, rectCartogramOptions);
    m_relPainting = std::make_shared<RELPainting>(m_relPtr, m_rectangularDual);

    // auto constructedSTgraphs = m_rectangularDual->buildSTGraphsFromREL(*m_relPtr);
    // auto constructedDuals = m_rectangularDual->buildDualsFromREL(*m_relPtr);

    //bool buildstatus =  m_rectangularDual->buildSTandDUal(m_rel);

    //std::cout << "Build st and dual stats: " << buildstatus << std::endl;

    // std::cout << "ST graph construction status: " << constructedSTgraphs << std::endl;
    // std::cout << "Dual graphs construction status: " << constructedDuals << std::endl;

    // m_rectangularDual->debugListUnassignedHalfEdges(*m_relPtr, RED);
    // m_rectangularDual->debugPrintFacesForColor(*m_relPtr, RED);
    // std::cout << " blue: " << std::endl;
    // m_rectangularDual->debugListUnassignedHalfEdges(*m_relPtr, BLUE);
    // m_rectangularDual->debugPrintFacesForColor(*m_relPtr, BLUE);


    bool computedMaximalSegments = m_rectangularDual->computeMaximalSegments(*m_relPtr);
    bool computedSegmentPositions = m_rectangularDual->computeSegmentPositions(*m_relPtr);
    bool computedRects = m_rectangularDual->computeRectanglesFromSegments(*m_relPtr);

    m_rectangularDual->fixRectangleAreas(*m_relPtr);


    // // inspect some vertices
    // for (int v = 0; v < m_relPtr->getVertices().size(); ++v) {
    //     const auto &V = m_relPtr->getVertices()[v];
    //     std::cout << "V[" << v << "] '" << V.label
    //               << "' left=" << V.left_segment
    //               << " right=" << V.right_segment
    //               << " bottom=" << V.bottom_segment
    //               << " top=" << V.top_segment << "\n";
    // }
    //
    // std::cout << "-------------------" << std::endl;
    //
    // auto maximalSegments = m_rectangularDual->getMaximalSegments();
    //
    // for (int s = 0; s < maximalSegments.size(); ++s) {
    //     const auto &seg = maximalSegments[s];
    //     std::cout << "segment " << s << " type=" << seg.type
    //               << " halfedges=" << seg.halfedges.size()
    //               << " incoming verts=" << seg.incoming_vertices.size()
    //               << " outgoing verts=" << seg.outgoing_vertices.size() << "\n";
    // }


    //    m_renderer->addPainting(m_debugPainting, "Debugging");

    //m_relPtr->flipEdgeDiagonally(12, false);

    m_renderer->addPainting(m_rectPainting, "RectangularCartogram");
    m_renderer->addPainting(m_relPainting, "REL");

}

void RectangularCartogramDemo::processData() {
    std::cout << "processing data" << std::endl;

    try {
        m_rel.buildFromJson(m_projectData); // will validate & throw if errors found
    } catch (const std::exception& e) {
        std::cerr << "Failed to load REL: " << e.what() << std::endl;
    }

    m_rel.printSummary();

    m_relPtr = std::make_shared<RegularEdgeLabeling>(m_rel);
    m_relPtr->setBoundingBox(BoundingBox{0, 1600, 0, 900});

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
    auto loadDataButton = new QPushButton("Load Data (json)");
    vLayout->addWidget(inputSettings);
    vLayout->addWidget(loadDataButton);

    auto* debugSettings = new QLabel("<h3>Debug settings</h3>", vWidget);
    m_showREL = new QCheckBox("Show REL");
    m_showREL->setChecked(true);
    vLayout->addWidget(debugSettings);
    vLayout->addWidget(m_showREL);

    // EDGE SELECTION/MANIPULATION BUTTONS
    auto* selectionLabel = new QLabel("<h3>Selection Actions</h3>", vWidget);
    auto* btnFlipColor = new QPushButton("Flip Color");
    auto* btnFlipDiagCW = new QPushButton("Flip Diagonal ▶ (CW)");
    auto* btnFlipDiagCCW = new QPushButton("Flip Diagonal ◀ (CCW)");
    auto* btnClearSelection = new QPushButton("Clear Selection");

    vLayout->addWidget(selectionLabel);
    vLayout->addWidget(btnFlipColor);
    vLayout->addWidget(btnFlipDiagCW);
    vLayout->addWidget(btnFlipDiagCCW);
    vLayout->addWidget(btnClearSelection);


    connect(loadDataButton, &QPushButton::clicked, [this, loadDataButton]() {
        QString startDir = "data";
        std::filesystem::path filePath = QFileDialog::getOpenFileName(this, tr("Select region data file"), startDir).toStdString();
        if (filePath.empty()) return;
        loadData(filePath);
        loadDataButton->setText(QString::fromStdString(filePath.filename().string()));
    });

    connect(m_showREL, &QCheckBox::toggled, [this, loadDataButton]() {
        m_relPainting->drawRel(m_showREL->isChecked());
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
        if (m_rectangularDual && m_relPtr->isValidREL()) {
            m_rectangularDual->computeMaximalSegments(*m_relPtr);
            m_rectangularDual->computeSegmentPositions(*m_relPtr);
            m_rectangularDual->computeRectanglesFromSegments(*m_relPtr);
        }
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
        if (m_rectangularDual && m_relPtr->isValidREL()) {
            m_rectangularDual->computeMaximalSegments(*m_relPtr);
            m_rectangularDual->computeSegmentPositions(*m_relPtr);
            m_rectangularDual->computeRectanglesFromSegments(*m_relPtr);
        }
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
        if (m_rectangularDual && m_relPtr->isValidREL()) {
            m_rectangularDual->computeMaximalSegments(*m_relPtr);
            m_rectangularDual->computeSegmentPositions(*m_relPtr);
            m_rectangularDual->computeRectanglesFromSegments(*m_relPtr);
        }
        m_renderer->update();
    });

    connect(m_renderer, &GeometryWidget::clicked, [this](Point<Inexact> pt){
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
        const auto &bb = *optbb;

        double hx = bb.right;
        double hy = bb.top;

        double wx = static_cast<double>(pt.x());
        double wy = static_cast<double>(pt.y());
        double dx = wx - hx;
        double dy = wy - hy;
        double d2 = dx * dx + dy * dy;

        std::cout << "checking distance for click" << std::endl;

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "checking distance for click\n";
        std::cout << " bb.right,bb.top = (" << hx << ", " << hy << ")\n";
        std::cout << " pt.x(),pt.y()   = (" << wx << ", " << wy << ")\n";
        std::cout << " dx,dy = (" << dx << "," << dy << ") d2=" << d2 << "\n";
        std::cout << " handle tolerance (squared) = " << (m_bboxHandleTolerance*m_bboxHandleTolerance) << "\n";
        if (d2 <= m_bboxHandleTolerance * m_bboxHandleTolerance) {
            m_bboxDragging = true;
            m_dragStartWorld = Point<Inexact>(wx, wy);
            m_bboxBeforeDrag = bb;
        }
    });

    // Update bounding box while dragging
    connect(m_renderer, &GeometryWidget::dragMoved, this, [this](Point<Inexact> pt) {
        if (!m_bboxDragging) return;
        if (!m_relPtr || !m_rectangularDual) return;

        //std::cout << "updating bb" << std::endl;

        double newRight = static_cast<double>(pt.x());
        double newTop = static_cast<double>(pt.y());
        double left = m_bboxBeforeDrag.left;
        double bottom = m_bboxBeforeDrag.bottom;

        //clamp to min size
        if (newRight < left + m_bboxMinWidth) newRight = left + m_bboxMinWidth;
        if (newTop   < bottom + m_bboxMinHeight) newTop   = bottom + m_bboxMinHeight;

        BoundingBox newbb;
        newbb.left = left;
        newbb.right = newRight;
        newbb.bottom = bottom;
        newbb.top = newTop;

        m_relPtr->setBoundingBox(newbb);

        m_rectangularDual->computeSegmentPositions(*m_relPtr);
        m_rectangularDual->computeRectanglesFromSegments(*m_relPtr);
        m_rectangularDual->fixRectangleAreas(*m_relPtr);


        m_renderer->update();
    });

    connect(m_renderer, &GeometryWidget::dragEnded, this, [this](const Point<Inexact> &pt) {
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