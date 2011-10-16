cmake_minimum_required (VERSION 2.6)
project (EmptyTab)

# Global Variables
if(MSVC)
    set(INCLUDE_DIR_PREFIX "C:/Golems" )
else ()
    set(INCLUDE_DIR_PREFIX /usr/local )
endif()

set ( SRC_TAB_SIMULATION EmptyTab EmptyTabApp )


set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})
  include_directories (../../src)
  include_directories (${INCLUDE_DIR_PREFIX}/include/dart)
  include_directories (${INCLUDE_DIR_PREFIX}/include/eigen3)

  if(MSVC)
	add_executable (EmptyTab WIN32 ${SRC_TAB_SIMULATION})
	SET_TARGET_PROPERTIES(EmptyTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
	#SET_TARGET_PROPERTIES(EmptyTab PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (EmptyTab  ${SRC_TAB_SIMULATION})
   endif()
   
   target_link_libraries (EmptyTab grip ${INCLUDE_DIR_PREFIX}/lib/dart ${wxWidgets_LIBRARIES} )
  
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

