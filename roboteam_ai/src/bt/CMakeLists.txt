# compile all files in bt directory
file(GLOB BT_SOURCES *.cpp *.hpp)

add_library(bt
        ${BT_SOURCES}
        )
add_dependencies(bt
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


