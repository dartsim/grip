# The name of our project is "librst"
# root source directory = ${LIBRST_SOURCE_DIR}
# root binary directory = ${LIBRST_BINARY_DIR}

cmake_minimum_required (VERSION 2.6)
project (librst)

set (CMAKE_VERBOSE_MAKEFILE OFF)
if(NOT MSVC)
  set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")
endif(NOT MSVC)

# ------------------------
# Added 2011-11-06 achq
set ( DART_PATH "/home/ana/Research/Grip/dart" )

set( DART_HEADERS "${DART_PATH}/src" )

set ( DART_LIBS collision
                dynamics
                geometry
                integration
                kinematics
                lcpsolver
                optimizer
                renderer
                utils
                yui )

link_directories( ${DART_PATH}/lib )

# ------------------------

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
../
${DART_HEADERS}
)

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories (${HEADERS})
  add_library (rst STATIC ${SRC})
  #add_executable (rst_exe ${SRC})
  #target_link_libraries (rst_exe ${wxWidgets_LIBRARIES})
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
