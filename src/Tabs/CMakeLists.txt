project(gtabs)

# Search all header and source files
file (GLOB tabs_srcs "*.cpp")
file (GLOB tabs_hdrs "*.h")

# Library 
set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
set (wxWidgets_USE_LIBS base core gl png tiff jpeg)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_library (gtabs ${tabs_srcs} ${tabs_hdrs})
  target_link_libraries (gtabs ggui gtools)
  set_target_properties (gtabs PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${tabs_hdrs} DESTINATION include/grip/Tabs)
  install (TARGETS gtabs ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
