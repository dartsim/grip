project(gvcollide)

#Search all header and source files
file (GLOB vcollide_srcs build.cpp
                         collide.cpp
                         NBody.cpp
                         overlap.cpp
                         PairData.cpp
                         RAPID.cpp
                         sample_client.cpp
                         VCollide.cpp
                         VInternal.cpp )

file (GLOB vcollide_hdrs AABB.H
                         EndPoint.H
                         matvec.H
                         moments.H
                         NBody.H
                         obb.H
                         overlap.H
                         PairData.H
                         RAPID.H
                         RAPID_private.H
                         RAPID_version.H
                         VCollide.h
                         VInternal.H )

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
