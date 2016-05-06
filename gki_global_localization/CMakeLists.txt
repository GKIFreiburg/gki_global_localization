cmake_minimum_required(VERSION 2.8.3)
project(gki_global_localization)
#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    tf)

catkin_package(
)

add_library(globloc
                    src/global_localization.h
                    src/global_localization.cc
                    src/map.c
                    src/map.h
                    src/map_cspace.cpp
                    src/laser.h
                    src/field_map.h
                    src/field_map.cc
                    src/likelihood_field.h
                    src/likelihood_field.cc)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(gki_global_localization src/glob_loc_main.cc)
add_executable(global_localize_once src/global_localize_once.cc)
target_link_libraries(gki_global_localization globloc
    ${catkin_LIBRARIES})
target_link_libraries(global_localize_once globloc
    ${catkin_LIBRARIES})
add_dependencies(gki_global_localization ${catkin_EXPORTED_TARGETS})

