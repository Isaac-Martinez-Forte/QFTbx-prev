# - Find the QTermWidget include and library dirs and define a some macros
#
# The module defines the following variables
#  QTERMWIDGET_FOUND         - Set to TRUE if all of the above has been found
#
#  QTERMWIDGET_INCLUDE_DIR   - The QTermWidget include directory
#
#  QTERMWIDGET_INCLUDE_DIRS  - The QTermWidget include directory
#
#  QTERMWIDGET_LIBRARIES     - The libraries needed to use QTermWidget
#
#  QTERMWIDGET_USE_FILE      - The variable QTERMWIDGET_USE_FILE is set which is the path
#                              to a CMake file that can be included to compile qtermwidget
#                              applications and libraries. It sets up the compilation
#                              environment for include directories and populates a
#                              QTERMWIDGET_LIBRARIES variable.
#
# QTERMWIDGET_QT_LIBRARIES   - The Qt libraries needed by QTermWidget
#
# Typical usage:
#    find_package(QTERMWIDGET5)
#
#    include(${QTERMWIDGET_USE_FILE})
#    add_executable(foo main.cpp)
#    target_link_libraries(foo ${QTERMWIDGET_QT_LIBRARIES} ${QTERMWIDGET_LIBRARIES})

set(QTERMWIDGET_INCLUDE_DIR   /home/isaacmf/Descargas/qterminal/qtermwidget/algo/include/qtermwidget5)
set(QTERMWIDGET_LIBRARY       qtermwidget5)

set(QTERMWIDGET_LIBRARIES     ${QTERMWIDGET_LIBRARY})
set(QTERMWIDGET_INCLUDE_DIRS  "${QTERMWIDGET_INCLUDE_DIR}")

set(QTERMWIDGET_USE_FILE      "${CMAKE_CURRENT_LIST_DIR}/qtermwidget5_use.cmake")
set(QTERMWIDGET_FOUND 1)

set(QTERMWIDGET_VERSION_MAJOR 0)
set(QTERMWIDGET_VERSION_MINOR 6)
set(QTERMWIDGET_VERSION_PATCH 0)
set(QTERMWIDGET_VERSION 0.6.0)

mark_as_advanced(QTERMWIDGET_LIBRARY QTERMWIDGET_INCLUDE_DIR)
