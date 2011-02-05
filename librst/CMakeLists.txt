# The name of our project is "librst"
# root source directory = ${LIBRST_SOURCE_DIR}
# root binary directory = ${LIBRST_BINARY_DIR}

cmake_minimum_required (VERSION 2.6)
project (librst)

set (CMAKE_VERBOSE_MAKEFILE OFF)
if(NOT MSVC)
  set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")
endif(NOT MSVC)

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
)

set ( SRC_FILES 
Tools/GLTools
Tools/Link
Tools/Model3DS
Tools/Object
Tools/Robot
Tools/World
Tools/IK
Tools/GL/GLTexture.cpp
Tools/Collision/build
Tools/Collision/collide
Tools/Collision/NBody
Tools/Collision/overlap
Tools/Collision/PairData
Tools/Collision/RAPID
Tools/Collision/sample_client
Tools/Collision/VCollide
Tools/Collision/VInternal
RST
)

set ( SRC 
${SRC_GUI}
${SRC_WX}
${SRC_FILES}
)

set ( HEADERS
./Tabs
./Tools/ANN
./Tools/Collision
./Tools/GL
./Tools
./GUI
./
../
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
