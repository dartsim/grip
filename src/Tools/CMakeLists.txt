project(gtools)

# Parser URDF
if ( PARSER_URDF )
  message( "Parser URDF ON -- Adding corresponding source and header files")
  file (GLOB tools_srcs Parser.cpp
                   ParserUtilities.cpp
	           utils.cpp
                   ParserURDF.cpp )

  file (GLOB tools_hdrs Parser.h
                   ParserUtilities.h
	           utils.h
                   Constants.h
                   ParserURDF.h  )
else ( PARSER_URDF )
  message( "Parser URDF OFF -- No adding source and header files")
  file (GLOB tools_srcs Parser.cpp
                   ParserUtilities.cpp
	           utils.cpp )
  file (GLOB tools_hdrs Parser.h
                   ParserUtilities.h
	           utils.h
                   Constants.h  )
endif ( PARSER_URDF )


set (srcs
GL/glcommon.h)

# Library 
set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
set (wxWidgets_USE_LIBS base core gl png tiff jpeg)
find_package (wxWidgets)
if (wxWidgets_FOUND) 
  include (${wxWidgets_USE_FILE})
  add_library (gtools ${tools_srcs}  ${srcs} ${tools_hdrs})
  set_target_properties (gtools PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${tools_hdrs} DESTINATION include/grip/Tools)
  install (TARGETS gtools ARCHIVE DESTINATION lib)
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
