cmake_minimum_required (VERSION 2.6)

# # TinyXML++ CMake generate file

# # Search all header and source files
# file (GLOB ticpp_srcs "*.cpp")
# file (GLOB ticpp_hdrs "*.h")

# # Library 
# add_library (ticpp ${ticpp_srcs} ${ticpp_hdrs})
# set_target_properties (ticpp PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
# #install (FILES ${ticpp_hdrs} DESTINATION include/grip/ticpp)
# #install (TARGETS ticpp ARCHIVE DESTINATION lib)

# TinyXML++ CMake generate file

project(TiCpp)

set(SRC_FILES 
    ticpp.cpp
    tinystr.cpp
    tinyxml.cpp
    tinyxmlerror.cpp
    tinyxmlparser.cpp 
)

set(HEADER_FILES
    ticpp.h
    ticpprc.h
    tinystr.h
    tinyxml.h 
)

add_definitions(-DTIXML_USE_TICPP)

add_library(ticpp ${SRC_FILES})

set_target_properties (ticpp PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
install (FILES ${HEADER_FILES} DESTINATION include/grip/ticpp)
install (TARGETS ticpp ARCHIVE DESTINATION lib)