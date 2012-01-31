cmake_minimum_required (VERSION 2.6)
project (tabPalletizing)

# include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
# include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)
# This include is used only if you have installed Grip

# Search all header and source files
file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
	add_executable (tabPalletizing WIN32 ${srcs} ${hdrs})
	set_target_properties (tabPalletizing PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (tabPalletizing ${srcs} ${hdrs})
   endif()
   
   target_link_libraries (tabPalletizing ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES} )
  
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
