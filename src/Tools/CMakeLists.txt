project(gtools)

file (GLOB tools_srcs 
  utils.cpp )

file (GLOB tools_hdrs
  utils.h
  Constants.h )

set (srcs
GL/glcommon.h)

# Library 
set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
set (wxWidgets_USE_LIBS base core gl png tiff jpeg)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_library (gtools ${tools_srcs}  ${srcs} ${tools_hdrs})
  set_target_properties (gtools PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${tools_hdrs} DESTINATION include/grip/Tools)
  install (TARGETS gtools ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
