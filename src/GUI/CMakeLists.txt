project(ggui)

# Search all header and source files
file (GLOB gui_srcs "*.cpp")
file (GLOB gui_hdrs "*.h")

# Library 
set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_library (ggui ${gui_srcs} ${gui_hdrs})
  target_link_libraries (ggui gtools gtabs 3dbpp tinyxml2)
  set_target_properties (ggui PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${gui_hdrs} DESTINATION include/grip/GUI)
  install (TARGETS ggui ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)