cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

project(qterminal)

include(GNUInstallDirs)

set(STR_VERSION "0.7.0")
#set(LXQTBT_MINIMUM_VERSION "0.1.0")


# additional cmake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. C++11 support is required")
endif()

# we need qpa/qplatformnativeinterface.h for global shortcut
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5LinguistTools REQUIRED)
if(APPLE)
elseif(UNIX)
    find_package(Qt5X11Extras REQUIRED)
endif()
#find_package(QTermWidget5 REQUIRED)
#find_package(lxqt-build-tools ${LXQTBT_MINIMUM_VERSION} REQUIRED)
#include(LXQtTranslateTs)
#message(STATUS "Qt version: ${Qt5Core_VERSION_STRING}")

#include(${QTERMWIDGET_USE_FILE})

# TODO remove Qxt
message(STATUS "Using bundled Qxt...")
set(QXT_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src/third-party")


if(APPLE)
   find_library(CARBON_LIBRARY Carbon REQUIRED)
   message(STATUS "CARBON_LIBRARY: ${CARBON_LIBRARY}")
elseif(UNIX)
   find_package(X11 REQUIRED)
   message(STATUS "X11_X11_LIB: ${X11_X11_LIB}")
endif()

add_definitions(-DSTR_VERSION=\"${STR_VERSION}\")
add_definitions(${Qt5X11Extras_DEFINITIONS})


set(EXE_NAME qterminal)

set(QTERM_SRC
    src/main.cpp
    src/mainwindow.cpp
    src/tabwidget.cpp
    src/termwidget.cpp
    src/termwidgetholder.cpp
    src/properties.cpp
    src/propertiesdialog.cpp
    src/bookmarkswidget.cpp
    src/fontdialog.cpp
)

set(QTERM_MOC_SRC
    src/mainwindow.h
    src/tabwidget.h
    src/termwidget.h
    src/termwidgetholder.h
    src/propertiesdialog.h
    src/bookmarkswidget.h
    src/fontdialog.h
)

if(NOT QXT_FOUND)
    set(QTERM_SRC ${QTERM_SRC} src/third-party/qxtglobalshortcut.cpp)
    set(QTERM_MOC_SRC ${QTERM_MOC_SRC} src/third-party/qxtglobalshortcut.h)

    if(WIN32)
        set(QTERM_SRC ${QTERM_SRC} src/third-party/qxtglobalshortcut_win.cpp)
    elseif(APPLE)
        set(QTERM_SRC ${QTERM_SRC} src/third-party/qxtglobalshortcut_mac.cpp)
    else()
        set(QTERM_SRC ${QTERM_SRC} src/third-party/qxtglobalshortcut_x11.cpp)
    endif()
endif()

set(QTERM_UI_SRC
    src/forms/qterminal.ui
    src/forms/propertiesdialog.ui
    src/forms/bookmarkswidget.ui
    src/forms/fontdialog.ui
)

set(QTERM_RCC_SRC
    src/icons.qrc
)

qt5_wrap_ui( QTERM_UI ${QTERM_UI_SRC} )
qt5_wrap_cpp( QTERM_MOC ${QTERM_MOC_SRC} )
qt5_add_resources( QTERM_RCC ${QTERM_RCC_SRC} )

include_directories(
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}"
 #   ${QTERMWIDGET_INCLUDE_DIRS}
    ${QXT_INCLUDE_DIRS}
)
if(X11_FOUND)
    include_directories("${X11_INCLUDE_DIR}")
endif()


# TODO/FIXME: apple bundle
set(GUI_TYPE "")
set(APPLE_BUNDLE_SOURCES "")
if(APPLEBUNDLE)
    add_definitions(-DAPPLE_BUNDLE)
    set(GUI_TYPE MACOSX_BUNDLE)

    # create Info.plist file
    set(MACOSX_BUNDLE_ICON_FILE qterminal.icns)
    set(MACOSX_BUNDLE_INFO_STRING "QTerminal ${STR_VERSION}")
    set(MACOSX_BUNDLE_GUI_IDENTIFIER "com.qterminal")
    set(MACOSX_BUNDLE_LONG_VERSION_STRING "${STR_VERSION}")
    set(MACOSX_BUNDLE_BUNDLE_NAME "${EXE_NAME}")
    set(MACOSX_BUNDLE_SHORT_VERSION_STRING "${STR_VERSION}")
    set(MACOSX_BUNDLE_BUNDLE_VERSION "${STR_VERSION}")
    set(MACOSX_BUNDLE_COPYRIGHT "(c) Petr Vanek &lt;petr@yarpen.cz&gt;")

    set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/macosx/qterminal.icns"
        PROPERTIES MACOSX_PACKAGE_LOCATION Resources
    )
    # use icon for app bundle to be visible in finder
    set(APPLE_BUNDLE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/macosx/qterminal.icns")
else()
    set(TRANSLATIONS_DIR "${CMAKE_INSTALL_FULL_DATADIR}/qterminal/translations")
    add_definitions(-DTRANSLATIONS_DIR=\"${TRANSLATIONS_DIR}\")
endif()

add_executable(${EXE_NAME} ${GUI_TYPE}
    ${QTERM_SRC}
    ${QTERM_UI}
    ${QTERM_MOC}
    ${QTERM_RCC}
    ${APPLE_BUNDLE_SOURCES}
    ${QTERM_QM}
)

set_property(TARGET ${EXE_NAME} APPEND PROPERTY
    INTERFACE_AUTOUIC_OPTIONS
)

qt5_use_modules(${EXE_NAME} Core Widgets Gui LinguistTools)

include(ExternalProject)

ExternalProject_Add(QTermWidget5
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/Library/qtermwidget
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/qtermwidget
)

include_directories("${CMAKE_BINARY_DIR}/qtermwidget/include/qtermwidget5/")

add_dependencies(${EXE_NAME} QTermWidget5)
target_link_libraries(${EXE_NAME} "${CMAKE_BINARY_DIR}/qtermwidget/lib64/libqtermwidget5.a")

if(QXT_FOUND)
    target_link_libraries(${EXE_NAME} ${QXT_CORE_LIB} ${QXT_GUI_LIB})
endif()

if(APPLE)
    target_link_libraries(${EXE_NAME} ${CARBON_LIBRARY})
elseif(UNIX)
    target_link_libraries(${EXE_NAME} Qt5::X11Extras)
endif()

if(X11_FOUND)
    target_link_libraries(${EXE_NAME} ${X11_X11_LIB})
endif()


install(FILES
    qterminal.desktop
    qterminal_drop.desktop
    DESTINATION "${CMAKE_INSTALL_DATADIR}/applications"
)

install(FILES
    qterminal.appdata.xml
    DESTINATION "${CMAKE_INSTALL_DATADIR}/appdata"
)

if(NOT APPLEBUNDLE)
    install(TARGETS ${EXE_NAME} RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    install(FILES ${QTERM_QM} DESTINATION ${TRANSLATIONS_DIR})
    install(FILES src/icons/qterminal.png DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/64x64/apps")
else()
    message(STATUS "APPLEBUNDLE")

    install(CODE "message(STATUS \"Cleaning previously installed bundle (rm -r)\")")
    install(CODE "execute_process(COMMAND rm -r ${CMAKE_INSTALL_PREFIX}/${EXE_NAME}.app)")

    install(TARGETS ${EXE_NAME} DESTINATION "${CMAKE_INSTALL_PREFIX}")

    # helper stuff to create real apple bundle.
    # Black magic is summoned here...
    if(APPLEBUNDLE_STANDALONE)
        message(STATUS "APPLEBUNDLE_STANDALONE")
        configure_file("${CMAKE_SOURCE_DIR}/bundle.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/bundle.cmake" @ONLY)
        install(SCRIPT "${CMAKE_SOURCE_DIR}/bundle.cmake")
    endif()

    # bundle required keytabs from the qtermwidget package as well
    install(CODE "message(STATUS \"Bundling (cp) keytab files from ${QTERMWIDGET_SHARE}/qtermwidget/\")")
    install(CODE "execute_process(COMMAND cp -r ${QTERMWIDGET_SHARE}/qtermwidget/ ${CMAKE_INSTALL_PREFIX}/${EXE_NAME}.app/Contents/Resources)")

    install(FILES ${QTERM_QM} DESTINATION ${CMAKE_INSTALL_PREFIX}/${EXE_NAME}.app/Contents/translations)
endif()


# make lupdate
# it generates new translation files
add_custom_target(lupdate
    ${QT_QMAKE_EXECUTABLE} -project -o "${CMAKE_CURRENT_BINARY_DIR}/qterminal.pro"
    COMMAND ${QT_LUPDATE_EXECUTABLE} "${CMAKE_CURRENT_BINARY_DIR}/qterminal.pro"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
