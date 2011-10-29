project(gvcollide)

#Search all header and source files
file (GLOB vcollide_srcs "*.cpp")
file (GLOB vcollide_hdrs "*.h")

# Library 
set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_library (gvcollide ${vcollide_srcs} ${vcollide_hdrs})
  set_target_properties (gvcollide PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${vcollide_hdrs} DESTINATION include/grip/VCollide)
  install (TARGETS gvcollide ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
