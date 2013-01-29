project(grip)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")

# Search all header and source files
file(GLOB_RECURSE grip_srcs "*.cpp")
file(GLOB_RECURSE grip_hdrs "*.h")

# Setup directory structure for Visual Studio
set(SUBDIRS GUI Tabs Tools)
foreach(SUBDIR ${SUBDIRS})
    file(GLOB_RECURSE dir_files ${SUBDIR}/*.h ${SUBDIR}/*.cpp)
    source_group(${SUBDIR} FILES ${dir_files})
endforeach()

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)

if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories(${DARTExt_INCLUDEDIR})
  include_directories(${DART_INCLUDEDIR})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${WXWIDGETS_SHARED_LIBRARY_DIRS})

  # Library
  add_library(grip ${grip_srcs} ${grip_hdrs})
  target_link_libraries(grip ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES})
  set_target_properties(grip PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
      ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  if(MSVC)
      set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
  endif()
  link_directories(${DARTExt_LIBRARY_DIRS})
  link_directories(${DART_LIBRARY_DIRS})
endif()

install(DIRECTORY ./ DESTINATION include/grip 
        FILES_MATCHING PATTERN "*.h*" 
        PATTERN "*CMake*" EXCLUDE)
        
install(TARGETS grip DESTINATION lib)
