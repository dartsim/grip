cmake_minimum_required (VERSION 2.6)
project (TemplateTab)

include_directories(../RST)

set (CMAKE_CXX_FLAGS "-L../RST")

set ( SRC_TAB_SIMULATION TemplateTab )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_executable (TemplateTab  ${SRC_TAB_SIMULATION})
  target_link_libraries (TemplateTab RST ${wxWidgets_LIBRARIES})
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

