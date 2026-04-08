set(SOURCES
		rectangular_cartogram_painting.cpp
		rectangular_dual.cpp
		regular_edge_labeling.cpp
        rel_painting.cpp
        demers.cpp
		layout_guide.cpp

)
set(HEADERS
		geometry_types.h
		rectangular_cartogram_painting.h
		rectangular_dual.h
		regular_edge_labeling.h
		rel_painting.h
        demers.h
		layout_guide.h
)

add_library(rectangular_cartogram ${SOURCES})
target_link_libraries(rectangular_cartogram
	PUBLIC 
	cartocrow::core
	${GLPK_LIBRARY}
)

#cartocrow_install_module(<module_name>)
install(FILES ${HEADERS} DESTINATION ${CARTOCROW_INSTALL_DIR}/rectangular_cartogram)