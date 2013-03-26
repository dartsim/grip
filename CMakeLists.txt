cmake_minimum_required (VERSION 2.8)
project (grip_all)

message (STATUS "Creating Makefile for the GRIP project")

set(GRIP_MAJOR_VERSION "2")
set(GRIP_MINOR_VERSION "4")
set(GRIP_PATCH_VERSION "0precise")
set(PKG_DESC "GRIP is a robotics simulator based on the DART physics engine.")
set(GRIP_VERSION "${GRIP_MAJOR_VERSION}.${GRIP_MINOR_VERSION}.${GRIP_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "dart, flann, ccd, fcl")

find_package(OpenGL)
message(STATUS "OpenGL libraries: ${OPENGL_LIBRARY}")
link_libraries(grip ${OPENGL_LIBRARIES})

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
       FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(MSVC)
  message(STATUS "Setup Visual Studio Specific Flags")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc /arch:SSE2 /openmp")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
elseif(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
  option(ENABLE_MSSE2 "Enable -msse2 flag" ON)
else()
  message(SEND_ERROR "Compiler not supported.")
endif()

set(CMAKE_DEBUG_POSTFIX "d")

# System Install paths
if(WIN32) 
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
elseif(APPLE)
  set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Install prefix" FORCE)
else()
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install prefix" FORCE)
endif()

###############
# Generate config file
configure_file(${CMAKE_SOURCE_DIR}/cmake/GRIPConfig.cmake.in ${CMAKE_SOURCE_DIR}/cmake/GRIPConfig.cmake @ONLY)
install(FILES ${CMAKE_SOURCE_DIR}/cmake/GRIPConfig.cmake DESTINATION share/grip)

configure_file(${CMAKE_SOURCE_DIR}/cmake/grip.pc.in ${CMAKE_SOURCE_DIR}/grip.pc @only)
install(FILES ${CMAKE_SOURCE_DIR}/grip.pc DESTINATION lib/pkgconfig)

# Add an "uninstall" target
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in" "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

###############
# Build Source
include_directories(BEFORE src)
set(GRIP_INCLUDEDIR src)
set(GRIP_LIBRARY_DIRS ${CMAKE_SOURCE_DIR}/lib)
set(GRIP_LIBS grip)
set(GRIP_LIBRARIES optimized ${GRIP_LIBS} debug ${GRIP_LIBS}d)
add_subdirectory(src)
add_subdirectory(grip-run)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "grip")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${GRIP_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${GRIP_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${GRIP_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "dart (>=2.3.0), libwxgtk2.8-dev")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Golems (http://github.com/golems/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Humanoids Robotics Lab at GT")

include(CPack)

