cmake_minimum_required (VERSION 2.6)
project (grip-run)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)

# Search all header and source files
file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")

if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)

find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories(${DARTExt_INCLUDE_DIRS})
  include_directories(${DART_INCLUDE_DIRS})
  include_directories(${GRIP_INCLUDE_DIRS})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${DART_LIBRARY_DIRS} ${GRIP_LIBRARY_DIRS} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
	add_executable (grip-run WIN32 ${srcs} ${hdrs})
	set_target_properties (grip-run PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (grip-run ${srcs} ${hdrs})
   endif()
   
   target_link_libraries (grip-run ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${GRIP_LIBRARIES} ${wxWidgets_LIBRARIES})
  
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

install(TARGETS grip-run DESTINATION bin)
