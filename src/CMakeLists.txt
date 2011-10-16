cmake_minimum_required (VERSION 2.6)
project (libgrip)

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

set( SRC_GUI 
GUI/Viewer
)

set( SRC_WX
Tabs/AllTabs
Tabs/InspectorTab
GUI/GUI
GUI/GRIPFrame
GUI/GRIPTimeSlice
GUI/GRIPSlider
GUI/TreeView
GUI/Viewer
GUI/DrawWorld
)

set ( SRC_FILES 
Tools/GLTools
Tools/GL/GLTexture.cpp
Tools/Parser
GRIP
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
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})
  include_directories (${HEADERS})
  include_directories (${INCLUDE_DIR_PREFIX}/include/dart)
  include_directories (${INCLUDE_DIR_PREFIX}/include/eigen3)
  add_library (grip STATIC ${SRC})
  #add_executable (grip_exe ${SRC})
  #target_link_libraries (grip_exe ${wxWidgets_LIBRARIES} dart)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
