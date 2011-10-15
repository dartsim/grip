cmake_minimum_required (VERSION 2.6)
project (EmptyTab)

# Global Variables
if(MSVC)
    set(INCLUDE_DIR_PREFIX "C:/Golems" )
else ()
    set(INCLUDE_DIR_PREFIX /usr/local )
endif()

set (CMAKE_CXX_FLAGS "-L../../libgrip -L{INCLUDE_DIR_PREFIX}/lib" )

set ( SRC_TAB_SIMULATION EmptyTab EmptyTabApp )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories (../../src)
  include_directories (${INCLUDE_DIR_PREFIX}/include/dart)
  include_directories (${INCLUDE_DIR_PREFIX}/include/eigen3)
  add_executable (EmptyTab  ${SRC_TAB_SIMULATION})
  target_link_libraries (EmptyTab grip dart ${wxWidgets_LIBRARIES} )
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

