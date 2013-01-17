cmake_minimum_required (VERSION 2.6)
project (tabEmpty)

# include (${CMAKE_INSTALL_PREFIX}/share/dart/DARTConfig.cmake)
# include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIPConfig.cmake)
# This include is used only if you have installed Grip


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
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
	add_executable (tabEmpty WIN32 ${srcs} ${hdrs})
	set_target_properties (tabEmpty PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (tabEmpty ${srcs} ${hdrs})
   endif()
   
   target_link_libraries (tabEmpty ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES} ${Boost_LIBRARIES})
  
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
