set(SOURCES
    rectangular_cartogram_gui.cpp
    persistent_settings.cpp
)

add_executable(rectangular_cartogram_gui ${SOURCES})

find_package(nlohmann_json REQUIRED)

target_link_libraries(
    rectangular_cartogram_gui
    PRIVATE
    cartocrow::core
    cartocrow::reader
    cartocrow::renderer
    CGAL::CGAL
    rectangular_cartogram
    nlohmann_json::nlohmann_json
)

install(TARGETS rectangular_cartogram_gui DESTINATION ${INSTALL_BINARY_DIR})