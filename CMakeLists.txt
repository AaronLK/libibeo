cmake_minimum_required( VERSION 2.8 )

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_subdirectory( libibeo )
add_subdirectory( utils )
