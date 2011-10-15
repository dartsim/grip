cmake_minimum_required (VERSION 2.6)
project (EmptyTab)

# -----------------------------------------------
set( DART_PATH "/home/ana/Research/Grip/dart")
set( DART_HEADERS "${DART_PATH}/src" )
set( DART_LIBS planning kinematics utils)

link_directories( ${DART_PATH}/lib )
# ------------------------------------------------

include_directories(../../librst ../../ ${DART_HEADERS} "${DART_HEADERS}/planning/external")

set (CMAKE_CXX_FLAGS "-L../../librst")

set ( SRC_TAB_SIMULATION EmptyTab EmptyTabApp )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_executable (EmptyTab  ${SRC_TAB_SIMULATION})
  target_link_libraries (EmptyTab rst ${wxWidgets_LIBRARIES} ${DART_LIBS})
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

