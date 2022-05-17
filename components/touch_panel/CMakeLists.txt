#file(GLOB_RECURSE SOURCES ./gsl2308/*.c)
idf_component_register(SRC_DIRS "gsl2308" 
                    INCLUDE_DIRS "gsl2308"
                    )

#target_link_libraries(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/libtouch_panel.a")
