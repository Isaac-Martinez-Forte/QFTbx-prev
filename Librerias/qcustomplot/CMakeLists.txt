cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)


project(qcustomplot)

if( WIN32 )
    set (CMAKE_PREFIX_PATH "C:\\Qt\\5.11.0\\msvc2015\\")
    #set (CMAKE_PREFIX_PATH "C:\\Qt\\5.11.0\\mingw53_32\\")
endif()

#Activamos Qt5
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)


FIND_PACKAGE(Qt5Widgets REQUIRED)
FIND_PACKAGE(Qt5PrintSupport REQUIRED)
FIND_PACKAGE(Qt5Gui REQUIRED)

add_library (qcustomplot STATIC qcustomplot.cpp qcustomplot.h)

target_link_libraries( qcustomplot Qt5::Core Qt5::Widgets Qt5::PrintSupport Qt5::Gui)


install(TARGETS qcustomplot
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Install headers
########################################################################
file(GLOB qcustomplot.h "${CMAKE_SOURCE_DIR}/*.h")
install(
    FILES qcustomplot.h
    DESTINATION include
)