# The name of our project is "librst"
# root source directory = ${LIBRST_SOURCE_DIR}
# root binary directory = ${LIBRST_BINARY_DIR}

cmake_minimum_required (VERSION 2.6)
project (librst)

# Global Variables
if(MSVC)
    set(INCLUDE_DIR_PREFIX "C:/Golems" )
else ()
    set(INCLUDE_DIR_PREFIX /usr/local )
endif()

set (CMAKE_VERBOSE_MAKEFILE OFF)
if(NOT MSVC)
  set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")
endif(NOT MSVC)
set (CMAKE_CXX_FLAGS -L{INCLUDE_DIR_PREFIX}/lib )

set( SRC_GUI 
GUI/Viewer
)

set( SRC_WX
Tabs/AllTabs
Tabs/InspectorTab
GUI/GUI
GUI/RSTFrame
GUI/RSTimeSlice
GUI/RSTSlider
GUI/TreeView
GUI/Viewer
GUI/DrawWorld
)

set ( SRC_FILES 
Tools/GLTools
Tools/GL/GLTexture.cpp
Tools/Parser
RST
)

set ( SRC 
${SRC_GUI}
${SRC_WX}
${SRC_FILES}
)

set ( HEADERS
./Tabs
./Tools/GL
./Tools
./GUI
./
)

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories (${HEADERS})
  include_directories (${INCLUDE_DIR_PREFIX}/include/dart)
  include_directories (${INCLUDE_DIR_PREFIX}/include/eigen3)
  add_library (rst STATIC ${SRC})
  #add_executable (rst_exe ${SRC})
  #target_link_libraries (rst_exe ${wxWidgets_LIBRARIES} dart)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
