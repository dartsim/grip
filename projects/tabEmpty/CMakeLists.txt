cmake_minimum_required (VERSION 2.6)
project (tabEmpty)

# Global Variables
if(MSVC)
    set(INCLUDE_DIR_PREFIX "C:/Golems" )
else ()
    set(INCLUDE_DIR_PREFIX /usr/local )
endif()

set ( SRC_TAB_SIMULATION tabEmpty tabEmptyApp )


set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS}
                   ${INCLUDE_DIR_PREFIX}/lib )
  include_directories (../../src)
  include_directories (${INCLUDE_DIR_PREFIX}/include/dart)
  include_directories (${INCLUDE_DIR_PREFIX}/include/eigen3)

  if(MSVC)
	add_executable (tabEmpty WIN32 ${SRC_TAB_SIMULATION})
	SET_TARGET_PROPERTIES(tabEmpty PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
	#SET_TARGET_PROPERTIES(tabEmpty PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (tabEmpty  ${SRC_TAB_SIMULATION})
   endif()
   
   target_link_libraries (tabEmpty grip dart ${wxWidgets_LIBRARIES} )
  
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

