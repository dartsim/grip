cmake_minimum_required (VERSION 2.6)
project (EmptyTab)

include_directories(../RST)

set (CMAKE_CXX_FLAGS "-L../RST")

set ( SRC_TAB_SIMULATION EmptyTab )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_executable (EmptyTab  ${SRC_TAB_SIMULATION})
  target_link_libraries (EmptyTab RST ${wxWidgets_LIBRARIES})
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

