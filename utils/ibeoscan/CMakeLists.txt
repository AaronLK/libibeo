project( ibeoscan )

cmake_minimum_required( VERSION 2.8 )

set( ibeoscan_SOURCES
  main.cpp
)

add_executable( ibeoscan ${ibeoscan_SOURCES} )

target_link_libraries( ibeoscan ibeo )

