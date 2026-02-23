set(SOURCES
        rel_painting.cpp
		rel_map.cpp
)
set(HEADERS
		rel_painting.h
		rel_map.h
)

add_library(rectangular_cartogram ${SOURCES})
target_link_libraries(rectangular_cartogram
	PUBLIC 
	cartocrow::core
)

#cartocrow_install_module(<module_name>)
install(FILES ${HEADERS} DESTINATION ${CARTOCROW_INSTALL_DIR}/rectangular_cartogram)