project(ggui)

# Search all header and source files
file (GLOB gui_srcs "*.cpp")
file (GLOB gui_hdrs "*.h")

# Library 
set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
set (wxWidgets_USE_LIBS base core gl png tiff jpeg)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
	include_directories( ${DART_INCLUDE_DIRS}  )
  add_library (ggui ${gui_srcs} ${gui_hdrs} ${DART_LIBRARIES} )
  target_link_libraries (ggui gtools gtabs)
  set_target_properties (ggui PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${gui_hdrs} DESTINATION include/grip/GUI)
  install (TARGETS ggui ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
