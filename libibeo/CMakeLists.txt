project( ibeo )

cmake_minimum_required( VERSION 2.8 )

set( ibeo_SOURCES
  libibeo.cpp
  include/message_types.h
  include/libibeo.h
)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package( Boost COMPONENTS system thread )

include_directories( ${Boost_INCLUDE_DIRS} )

add_library( ibeo SHARED ${ibeo_SOURCES} )

target_link_libraries( ibeo ${Boost_LIBRARIES} )

target_include_directories( ibeo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>
)
