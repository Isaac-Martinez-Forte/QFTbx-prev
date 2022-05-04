cmake_minimum_required(VERSION 3.4.3 FATAL_ERROR)


project(QFTbx)

set(EXE_NAME QFTbx)

include(GNUInstallDirs)

# The project version number.
set(VERSION_MAJOR   0   CACHE STRING "1.")
set(VERSION_MINOR   0   CACHE STRING "0.")
set(VERSION_PATCH   1   CACHE STRING "0")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

#Opciones de configuracion

OPTION (USE_CLANG "Use CLANG" OFF)
OPTION (USE_OpenMP "Use OpenMP" OFF)
OPTION (USE_CUDA "Use CUDA" OFF)
OPTION (USE_OpenCL "Use OpenCL" OFF)
OPTION (USE_Doxygen "Use Doxygen" OFF)
OPTION (USE_QML "Use QML" OFF)

if( WIN32 )
    #set (CMAKE_PREFIX_PATH "C:\\Qt\\5.11.0\\msvc2015\\")
    set (CMAKE_PREFIX_PATH "C:\\Qt\\5.11.1\\mingw53_32\\")
endif()

#CLang
IF(USE_CLANG)
    SET (CMAKE_C_FLAGS_INIT                "-Wall -std=c99")
    SET (CMAKE_C_FLAGS_DEBUG_INIT          "-g")
    SET (CMAKE_C_FLAGS_MINSIZEREL_INIT     "-Os -DNDEBUG")
    SET (CMAKE_C_FLAGS_RELEASE_INIT        "-O3 -DNDEBUG")
    SET (CMAKE_C_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")

    SET (CMAKE_CXX_FLAGS_INIT                "-Wall -std=c++0x -stdlib=libc++")
    SET (CMAKE_CXX_FLAGS_DEBUG_INIT          "-g")
    SET (CMAKE_CXX_FLAGS_MINSIZEREL_INIT     "-Os -DNDEBUG")
    SET (CMAKE_CXX_FLAGS_RELEASE_INIT        "-O3 -DNDEBUG")
    SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT "-O2 -g")
    SET (CMAKE_EXE_LINKER_FLAGS_INIT         "")
ENDIF()

#Opciones del compilador
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -march=native")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native -msse -msse2 -msse3 -mmmx")

set(CMAKE_BUILD_TYPE Debug)

IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    ADD_DEFINITIONS(-DOpenMP_AVAILABLE=1)
  ENDIF()
ENDIF()


IF (USE_CUDA)
    #set(CUDA_HOST_COMPILER /usr/bin/gcc53)
    FIND_PACKAGE(CUDA)
    if(CUDA_FOUND)
        list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O3;-DVERBOSE")
        SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
        set(CUDA_SEPARABLE_COMPILATION ON)
        FILE(GLOB QFTbx_CUDA GPU/CUDA/bnd_cuda.cu GPU/CUDA/e_hull_cuda.cu)
        ADD_DEFINITIONS(-DCUDA_AVAILABLE=1)

        CUDA_ADD_LIBRARY(CUDA STATIC ${QFTbx_CUDA})
    endif()
ENDIF()


# Handle OpenCL
IF (USE_OpenCL)
	find_package(OpenCL)
	if(OpenCL_FOUND)
		include_directories(${OpenCL_INCLUDE_DIRS})
		link_directories(${OpenCL_LIBRARY})
		ADD_DEFINITIONS(-DOpenCL_AVAILABLE=1)
	endif()
ENDIF()

#add_definitions(-DDEBUG=1)

# add a target to generate API documentation with Doxygen
if (USE_Doxygen)
    find_package(Doxygen)
    option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})
    
    if(BUILD_DOCUMENTATION)
        if(NOT DOXYGEN_FOUND)
            message(FATAL_ERROR "Doxygen is needed to build the documentation.")
        endif()
        set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
        set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)
        configure_file(${doxyfile_in} ${doxyfile})  
        add_custom_target(doc1
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
        add_custom_target(doc
        COMMAND make -f ${CMAKE_CURRENT_BINARY_DIR}/doc/latex/Makefile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/latex
        COMMENT "Generating Latex documentation with TexLive "
        VERBATIM)
        add_dependencies(doc doc1)
    
    endif()
endif()


#Activamos Qt5
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)



#Buscamos los paquetes de Qt5
find_package(Qt5 COMPONENTS Core Widgets PrintSupport REQUIRED)

#find_package(Qt5Quick 5.3.0 REQUIRED)
#find_package(Qt5Qml 5.3.0 REQUIRED)
find_package(Qt5Test REQUIRED)
#find_package(Qt5QuickTest 5.3.0 REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

#Buscamos los ficheros fuente
set(QFTbx_SRC main.cpp
    Modelo/LoopShaping/NaturalIntervalExtension/natural_interval_extension.cpp
    Modelo/EstructuraSistema/funciontransferencia.cpp
    DAO/adaptadorplantadao.cpp
    Modelo/EstructuraSistema/sistema.cpp
    Modelo/EstructuraSistema/polosceros.cpp
    Modelo/EstructuraSistema/cpolinomios.cpp
    Modelo/EstructuraSistema/kganancia.cpp
    Modelo/EstructuraSistema/knganancia.cpp
    #Modelo/EstructuraSistema/sistemasimplificada.cpp
    #Modelo/EstructuraSistema/funciontransferenciasimplificada.cpp
    #Modelo/EstructuraSistema/poloscerossimplificada.cpp
    #Modelo/EstructuraSistema/kgananciasimplificada.cpp
    GUI/diagramabode.cpp
    GUI/windowsgeneral.cpp
    GUI/introducirplanta.cpp
    GUI/intomega.cpp
    GUI/intincertidumbre.cpp
    Modelo/controlador.cpp
    Modelo/EstructurasDatos/parlineedit.cpp
    DAO/adaptadoromegadao.cpp
    Modelo/Objetos/incertidumbre.cpp
    Modelo/Templates/templates.cpp
    DAO/adaptadortemplatedao.cpp
    Modelo/Herramientas/tools.cpp
    Modelo/Objetos/omega.cpp
    DAO/adaptadorbounddao.cpp
    GUI/viewbound.cpp
    XmlParser/xmlparsersave.cpp
    Modelo/EstructurasDatos/datosplanta.cpp
    XmlParser/xmlparserload.cpp
    DAO/fdao.cpp
    GUI/intdatosboundaries.cpp
    Modelo/EstructurasDatos/datosbound.cpp
    GUI/intespecificaciones.cpp
    DAO/adaptadorespecificacionesdao.cpp
    GUI/introducirecontr.cpp
    DAO/adaptadorcontroladordao.cpp
    Modelo/LoopShaping/algorithm_sachin.cpp
    Modelo/LoopShaping/algorithm_isaac.cpp
    #Modelo/LoopShaping/algorithm_isaac_hilo.cpp
    Modelo/EstructurasDatos/var.cpp
    Modelo/LoopShaping/algorithm_nandkishor.cpp
    Modelo/LoopShaping/algorithm_rambabu.cpp
    GUI/viewboundreun.cpp
    Modelo/Boundaries/boundaries.cpp
    Modelo/Boundaries/contour.cpp
    Consola/consola.cpp
    Modelo/EstructuraSistema/formatolibre.cpp
    GUI/intloopshaping.cpp
    Modelo/LoopShaping/loopshaping.cpp
    GUI/viewloopshaping.cpp
    Modelo/LoopShaping/DeteccionViolacionBoundaries/deteccionviolacionboundaries.cpp
    #Modelo/LoopShaping/TestDeteccion.cpp
    Modelo/LoopShaping/EstructuraDatos/listaordenada.cpp
    DAO/adaptadorloopshapingdao.cpp
    Modelo/EstructurasDatos/datosloopshaping.cpp
    GUI/introducirtemplates.cpp
    GUI/viewtemplates.cpp
    GUI/verboundaries.cpp
    Modelo/Boundaries/algoritmointerseccionlineal1D.cpp
    Modelo/Boundaries/algoritmointerseccionproyectivo2D.cpp
    Modelo/Boundaries/contour2.cpp
    Modelo/LoopShaping/EstructuraDatos/arbol_exp.cpp )

set (QFTbx_OpenCL GPU/OpenCL/e_hull_opencl.cl)

set (QFTbx_HED Modelo/EstructuraSistema/funciontransferencia.h
    Modelo/LoopShaping/NaturalIntervalExtension/natural_interval_extension.h
    DAO/adaptadorplantadao.h
    Modelo/EstructuraSistema/sistema.h
    Modelo/EstructuraSistema/polosceros.h
    Modelo/EstructuraSistema/cpolinomios.h
    Modelo/EstructuraSistema/kganancia.h
    Modelo/EstructuraSistema/knganancia.h
    #Modelo/EstructuraSistema/sistemasimplificada.h
    #Modelo/EstructuraSistema/funciontransferenciasimplificada.h
    #Modelo/EstructuraSistema/poloscerossimplificada.h
    #Modelo/EstructuraSistema/kgananciasimplificada.h
    GUI/diagramabode.h
    GUI/windowsgeneral.h
    GUI/introducirplanta.h
    DAO/plantadao.h
    GUI/intomega.h
    GUI/intincertidumbre.h
    Modelo/controlador.h
    Modelo/EstructurasDatos/parlineedit.h
    DAO/adaptadoromegadao.h
    DAO/omegadao.h
    Modelo/Objetos/incertidumbre.h
    Modelo/Templates/templates.h
    DAO/templatedao.h
    DAO/adaptadortemplatedao.h
    Modelo/Herramientas/tools.h
    Modelo/Objetos/omega.h
    DAO/bounddao.h
    DAO/adaptadorbounddao.h
    GUI/viewbound.h
    Modelo/EstructurasDatos/datosplanta.h
    DAO/dao.h
    DAO/fdao.h
    GUI/intdatosboundaries.h
    Modelo/EstructurasDatos/datosbound.h
    XmlParser/parserload.h
    XmlParser/parsersave.h
    GUI/intespecificaciones.h
    DAO/especificacionesdao.h
    DAO/adaptadorespecificacionesdao.h
    GUI/introducirecontr.h
    DAO/adaptadorcontroladordao.h
    DAO/controladordao.h
    Modelo/LoopShaping/algorithm_sachin.h
    Modelo/LoopShaping/algorithm_isaac.h
    #Modelo/LoopShaping/algorithm_isaac_hilo.h
    Modelo/EstructurasDatos/var.h
    Modelo/LoopShaping/algorithm_nandkishor.h
    Modelo/LoopShaping/algorithm_rambabu.h
    Modelo/LoopShaping/EstructuraDatos/arbol_exp.h
    GUI/viewboundreun.h
    Modelo/Boundaries/boundaries.h
    Modelo/Boundaries/contour.h
    Consola/consola.h
    Modelo/EstructuraSistema/formatolibre.h
    GUI/intloopshaping.h
    Modelo/LoopShaping/loopshaping.h
    GUI/viewloopshaping.h
    Modelo/LoopShaping/EstructuraDatos/avl.h
    Modelo/LoopShaping/EstructuraDatos/tripleta.h
    Modelo/LoopShaping/EstructuraDatos/nodo.h
    Modelo/LoopShaping/DeteccionViolacionBoundaries/deteccionviolacionboundaries.h
    #Modelo/LoopShaping/TestDeteccion.h
    Modelo/LoopShaping/EstructuraDatos/listaordenada.h
    DAO/loopshapingdao.h
    DAO/adaptadorloopshapingdao.h
    Modelo/EstructurasDatos/datosloopshaping.h
    GUI/introducirtemplates.h
    GUI/viewtemplates.h
    GUI/verboundaries.h
    Modelo/Boundaries/algoritmointerseccionlineal1D.h
    Modelo/Boundaries/algoritmointerseccionproyectivo2D.h
    Modelo/Boundaries/contour2.h 
    Modelo/LoopShaping/funcionescomunes.h )


#Buscamos los ficheros de recursos
qt5_add_resources(QFTbx_RES_SRC ${CMAKE_SOURCE_DIR}/Imagenes/imagenesPlanta.qrc )
        #${CMAKE_SOURCE_DIR}/GUI/QML/qml.qrc )

                                
#Buscamos los ficheros QML
#find_package(Qt5QuickCompiler)
#qtquick_compiler_add_resources(QFTbx_QML_SRC ${CMAKE_SOURCE_DIR}/GUI/QML/qml.qrc)

if (USE_QML)
    file(GLOB_RECURSE QFTbx_QML_SRC *.qml)
    # This part is same as in Angew's answer
    set(copiedQmls "")
    foreach(QmlFile ${QFTbx_QML_SRC})
    get_filename_component(nam ${QmlFile} NAME)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${nam}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${QmlFile} ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${QmlFile}
        COMMENT "Copying ${QmlFile}"
        VERBATIM
    )
    list(APPEND copiedQmls ${CMAKE_CURRENT_BINARY_DIR}/${nam})
    endforeach()

    add_custom_target(
    CopyQMLs ALL
    DEPENDS ${copiedQmls}
    )
endif()

if( WIN32 )
	set (QFTbx_HED ${QFTbx_HED} Librerias/qcustomplot/qcustomplot.h)
	set (QFTbx_SRC ${QFTbx_SRC} Librerias/qcustomplot/qcustomplot.cpp)
endif()

add_executable(${EXE_NAME} ${QFTbx_SRC} ${QFTbx_HED} ${QFTbx_RES_SRC} ${CUDA_OBJECTS} ${QFTbx_OpenCL})

set_target_properties(${EXE_NAME} PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_link_libraries(${EXE_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport) #Qt5::Qml Qt5::Quick


if(CUDA_FOUND)
    ADD_DEPENDENCIES(${EXE_NAME} CUDA)
    TARGET_LINK_LIBRARIES(${EXE_NAME} CUDA)
endif()

if(OpenCL_FOUND)
	target_link_libraries (${EXE_NAME} ${OpenCL_LIBRARY})
endif()

set_property(TARGET ${EXE_NAME} APPEND PROPERTY
    INTERFACE_AUTOUIC_OPTIONS
)

#Buscamos las librerías estáticas

include(ExternalProject)

############################################################################
#ExternalProject_Add(QTerminal
#  SOURCE_DIR ${CMAKE_SOURCE_DIR}/Librerias/qterminal
#  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/qterminal
#)

############################################################################
ExternalProject_Add(muParserX
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/Librerias/muparserx
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/muparserx
)

include_directories("${CMAKE_BINARY_DIR}/muparserx/include/")

add_dependencies(${EXE_NAME} muParserX)

target_link_libraries(${EXE_NAME} "${CMAKE_BINARY_DIR}/muparserx/lib/libmuparserx.a")

############################################################################
if( NOT WIN32 )
	ExternalProject_Add(QCustomPlot
	  SOURCE_DIR ${CMAKE_SOURCE_DIR}/Librerias/qcustomplot
	  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/qcustomplot
	)

	include_directories("${CMAKE_BINARY_DIR}/qcustomplot/include/")

	add_dependencies(${EXE_NAME} QCustomPlot)
	target_link_libraries(${EXE_NAME} "${CMAKE_BINARY_DIR}/qcustomplot/lib/libqcustomplot.a")
endif()

############################################################################
ExternalProject_Add(cxsc
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/Librerias/cxsc
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/cxsc
)

include_directories("${CMAKE_BINARY_DIR}/cxsc/include/")

add_dependencies(${EXE_NAME} cxsc)

if ( NOT WIN32 )

    target_link_libraries(${EXE_NAME} "${CMAKE_BINARY_DIR}/cxsc/lib/libcxsc.so")

else ()

    target_link_libraries(${EXE_NAME} "${CMAKE_BINARY_DIR}/cxsc/lib/libcxsc.a")

endif()


