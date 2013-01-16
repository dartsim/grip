# Edited for DART - AHQ 2012-08-07

cmake_minimum_required (VERSION 2.8)
project (grip_all)

message (STATUS "Creating Makefile for the GRIP project")

set(GRIP_MAJOR_VERSION "2")
set(GRIP_MINOR_VERSION "0")
set(GRIP_PATCH_VERSION "1")
set(PKG_DESC "GRIP is a robotics simulator based on the DART physics engine.")
set(GRIP_VERSION "${GRIP_MAJOR_VERSION}.${GRIP_MINOR_VERSION}.${GRIP_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "dart, flann, ccd, fcl")

###############
# Global Variables
set (CMAKE_VERBOSE_MAKEFILE OFF)

if(WIN32)
	set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
    message (STATUS "Setup Visual Studio Specific Flags")
	set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
	set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
	set (CMAKE_DEBUG_POSTFIX "d")
	set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
	set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
   set (CMAKE_INSTALL_PREFIX /usr )
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3 -std=c++0x")
endif()

include (${CMAKE_INSTALL_PREFIX}/share/dart/DARTConfig.cmake)
include_directories(${DART_INCLUDEDIR})

###############
include_directories (src)
add_subdirectory (src)

set (srcs src/GRIP.cpp)
set (hdrs src/GRIPApp.h)

if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_library (grip ${srcs} ${hdrs})
  target_link_libraries (grip ggui gtools gtabs )
  target_link_libraries (grip ${DART_LIBRARIES} )
  set_target_properties (grip PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${hdrs} DESTINATION include/grip)
  install (TARGETS grip ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)

message(STATUS "CMake root is ${CMAKE_ROOT}")

###############
# Generate config file
configure_file(${CMAKE_SOURCE_DIR}/cmake/GRIPConfig.cmake.in ${CMAKE_SOURCE_DIR}/GRIPConfig.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/GRIPConfig.cmake.in ${CMAKE_SOURCE_DIR}/FindGRIP.cmake @ONLY)
install(FILES ${CMAKE_SOURCE_DIR}/GRIPConfig.cmake DESTINATION share/grip)
install(FILES ${CMAKE_SOURCE_DIR}/FindGRIP.cmake DESTINATION ${CMAKE_ROOT}/Modules)

configure_file(${CMAKE_SOURCE_DIR}/cmake/grip.pc.in ${CMAKE_SOURCE_DIR}/grip.pc @only)
install(FILES ${CMAKE_SOURCE_DIR}/grip.pc DESTINATION lib/pkgconfig)

# Add an "uninstall" target
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in" "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

###############
# Build Projects
set(GRIP_HEADERS ${CMAKE_SOURCE_DIR}/src)
set(GRIP_LIBRARIES grip ggui gtabs gtools ${DART_LIBRARIES})
set(GRIP_LIBDIR ${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(projects/tabEmpty)
add_subdirectory(projects/tabTemplate)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "grip")
set(CPACK_SYSTEM_NAME "amd64")
set(CPACK_PACKAGE_VERSION_MAJOR ${GRIP_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${GRIP_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${GRIP_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "dart (>=2.0.0), freeglut3, freeglut3-dev, libboost-dev, flann (>=1.8), libccd (>=1.4.2), libfcl (>=0.2.7), libeigen3-dev")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Golems (http://github.com/golems/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Humanoids Robotics Lab at GT")

include(CPack)

