cmake_minimum_required(VERSION 3.0.2)
project(grabe_mapping)

add_compile_options(-std=gnu++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  message_filters
  nav_msgs
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  std_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

roslint_cpp()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_plugin
  CATKIN_DEPENDS message_filters nav_msgs roscpp roslint rqt_gui rqt_gui_cpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


####### Slam6D ######

# Set output directories for libraries and executables
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin )
# Set output directories for multi-config builds (like with MSVC)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${PROJECT_SOURCE_DIR}/bin )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/3rdparty/CMakeModules" ${CMAKE_MODULE_PATH})

include(CheckSymbolExists)
check_symbol_exists(mmap sys/mman.h HAVE_MMAP)
check_symbol_exists(mkstemp stdlib.h HAVE_MKSTEMP)
set(CMAKE_REQUIRED_DEFINITIONS -D_GNU_SOURCE)
check_symbol_exists(fallocate fcntl.h HAVE_FALLOCATE)
set(CMAKE_REQUIRED_DEFINITIONS)
if(HAVE_MMAP AND HAVE_MKSTEMP AND HAVE_FALLOCATE)
  add_definitions(-DWITH_MMAP_SCAN)
endif()

#include_directories(OPENGL_INCLUDE_DIR)
if(WIN32)
  # Tells the config system not to automatically select  which libraries to
  # link against. Normally if a compiler supports #pragma lib, then the
  # correct library build variant will be automatically selected and linked
  # against, simply by the act of including one of that library's headers.
  # This macro turns that feature off.
  add_definitions(-DBOOST_ALL_NO_LIB)
  # Forces all libraries that have separate source, to be linked as dll's
  # rather than static libraries on Microsoft Windows (this macro is used to
  # turn on __declspec(dllimport) modifiers, so that the compiler knows which
  # symbols to look for in a dll rather than in a static library).
  add_definitions(-DBOOST_ALL_DYN_LINK)
endif()

if(MINGW)
  # https://github.com/msys2/MINGW-packages/issues/5233
  set(Boost_NO_BOOST_CMAKE ON)
endif()

set(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.44" "1.44.0" "1.45.0" "1.45" "1.46" "1.46.1" "1.47.0" "1.47" "1.48" "1.50" "1.52" "1.53" "1.55" "1.56")
if(WIN32)
  # for some unknown reason no one variant works on all windows platforms
  set(Boost_DEBUG 1)
  find_package( Boost COMPONENTS serialization graph regex filesystem system thread chrono date_time program_options system REQUIRED)
else()
  find_package( Boost COMPONENTS serialization graph regex filesystem system thread date_time program_options system REQUIRED)
endif()

link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})

find_package(SuiteSparse REQUIRED)
include_directories(/usr/include/suitesparse)

find_package(ANN)
if (NOT ANN_FOUND)
  add_subdirectory("3rdparty/ann")
  set(ANN_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/ann/ann_1.1.1_modified/include")
  set(ANN_LIBRARIES ann)
endif()

find_package(Newmat)
if(NOT NEWMAT_FOUND)
  add_subdirectory("3rdparty/newmat")
  set(NEWMAT_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/newmat/newmat-10")
  set(NEWMAT_LIBRARIES newmat)
endif()

if(APPLE)
  find_package(OpenCV REQUIRED HINTS "/usr/local/opt/opencv3/share/OpenCV/")
else()
  if(WIN32)
    set(OpenCV_STATIC ON)
  endif()
endif()

if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")
  include("${OpenCV_DIR}/OpenCVConfig.cmake")
  set(ADDITIONAL_OPENCV_FLAGS
          "-DCV_MINOR_VERSION=${OpenCV_VERSION_MINOR} -DCV_MAJOR_VERSION=${OpenCV_VERSION_MAJOR}"
          CACHE STRING "OpenCV Version Defines)"
          )
  ## Include the standard CMake script
else()
  set(ADDITIONAL_OPENCV_FLAGS
          ""
          CACHE STRING "OpenCV Version Defines (BLUB)"
          )
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_OPENCV_FLAGS}")

#################################################
# Declare Options and modify build accordingly ##
#################################################

option(WITH_CGAL "Compile with CGAL support" ON)
option(WITH_GMP "Compile with GMP support" ON) # on Windows GMP is provided by MPIR
option(WITH_LIBZIP "Compile with libzip support" ON)
option(WITH_OPENGL "Compile with OpenGL support" ON)
option(WITH_OPENCV "Compile with OpenCV support" ON)
option(WITH_QT "Compile tools relying on QT (qtshow)" ON)
option(WITH_GLFW "Compile with GLFW support" ON)
option(WITH_FTGL "Compile with FTGL support" ON)
option(WITH_XMLRPC "Compile with XMLRPC support" ON)
option(WITH_EIGEN3 "Compile with eigen3 support" ON)
option(WITH_LIBCONFIG "Compile with libconfig support" ON)
option(WITH_ROS "Compile with ROS support" OFF)
option(WITH_PYTHON "Compile Python bindings" ON)
option(WITH_WXWIDGETS "Compile with wxwidgets support" ON)
option(WITH_OPENCV_NONFREE "Whether to use non-free (patent encumbered) OpenCV functionalities" OFF)
option(WITH_COMPACT_OCTREE "Whether to use the compact octree display ON/OFF" OFF)
option(WITH_GLEE "Whether to use OpenGL extensions, requires glee. ON/OFF" OFF)
option(WITH_LASLIB "Whether to build LASlib based scanio library" ON)
option(WITH_E57 "Build library for e57 file format support." OFF)

# cvblob is needed for thermo
if (WITH_OPENCV)
  find_package(OpenCV REQUIRED)
  if (OpenCV_VERSION VERSION_LESS 4)
  message(STATUS "building cvblob")
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_subdirectory(3rdparty/cvblob)
  link_directories(${PROJECT_SOURCE_DIR}/3rdparty/cvblob)
  endif()
endif()

## CUDA accelerated collision detection
option(WITH_CUDA "Whether to build CUDA accelerated collision detection tools" OFF)
if(WITH_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${FOUND_CUDA_NVCC_INCLUDE})

  #owerrite users input - need to be fixed
  #select all NVIDIA GPU arch which support unified memory (CUDA toolkit >= 6.0) and arch>=30
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_30,code=sm_30;-gencode arch=compute_35,code=sm_35;-gencode arch=compute_50,code=sm_50;-gencode arch=compute_52,code=sm_52)

  message("CUDA_NVCC_FLAGS = ${CUDA_NVCC_FLAGS}")

  message(STATUS "With CUDA accelerated collision detection")
else()
  message(STATUS "Without CUDA accelerated collision detection")
endif()


## RivLib
option(WITH_RIVLIB "Whether the RIEGL rivlib is present ON/OFF" OFF)

if(WITH_RIVLIB)
  message(STATUS "Compiling a scan IO for RXP files")

  set(RIEGL_DIR ${PROJECT_SOURCE_DIR}/3rdparty/riegl/)

  if(UNIX)
    set(RiVLib_USE_STATIC_RUNTIME ON)
  endif()

  find_package(RiVLib QUIET COMPONENTS scanlib HINTS "${RIEGL_DIR}/cmake/")

  if(${RiVLib_FOUND})
    include_directories(${RiVLib_INCLUDE_DIRS})
  else()
    # TODO: Remove this if nobody is using the old RiVLib anymore.
    #       Change QUIET to REQUIRED in the find_package call for RiVLib above.

    message(STATUS "Cannot find current RiVLib. Trying to build scan IO for RXP files with old scanlib.")

    add_definitions(-DWITH_OLD_RIVLIB)

    find_package(LibXml2 REQUIRED)
    include_directories(${LIBXML2_INCLUDE_DIR})

    include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
    if(WIN32)
      set(RiVLib_SCANLIB_LIBRARY ${RIEGL_DIR}libscanlib-mt.lib ${RIEGL_DIR}libctrllib-mt.lib ${RIEGL_DIR}libboost_system-mt-1_43_0-vns.lib ${LIBXML2_LIBRARIES})
    else()
      set(RiVLib_SCANLIB_LIBRARY ${RIEGL_DIR}libscanlib-mt-s.a ${RIEGL_DIR}libctrllib-mt-s.a ${RIEGL_DIR}libboost_system-mt-s-1_43_0-vns.a pthread ${LIBXML2_LIBRARIES})
    endif()
  endif()
else()
  message(STATUS "Do NOT compile a scan IO for RXP")
endif()

## Micro-Epsilon libraries
option(WITH_MICROEPSILONLIB "Whether the Micro-Epsilon libraries are present ON/OFF" OFF)

if(WITH_MICROEPSILONLIB)
  message(STATUS "Setting variables for Micro-Epsilon libraries")
  set(MICROEPSILON_PATH ${PROJECT_SOURCE_DIR}/3rdparty/microepsilon/)

  if (EXISTS "${MICROEPSILON_PATH}/libllt.so.0")
    if (EXISTS "${MICROEPSILON_PATH}/libmescan.so.0")
      set(MICROEPSILON_LIBRARY ${MICROEPSILON_PATH}libllt.so.0 ${MICROEPSILON_PATH}libmescan.so.0)
    else()
      message(STATUS "Cannot find Micro-Epsilon libraries. Provide libraries or disable WITH_MICROEPSILONLIB.")
    endif()
  else()
    message(STATUS "Cannot find Micro-Epsilon libraries. Provide libraries or disable WITH_MICROEPSILONLIB.")
  endif()
else()
  message(STATUS "Microepsilon libraries not present.")
endif()

option(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON)
if (WITH_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

option(WITH_METRICS "Whether to use time metrics. ON/OFF" OFF)

option(WITH_ADDONS "Whether to download and use addons to 3DTK. ON/OFF" OFF)
if(WITH_ADDONS)
  message(STATUS "Compiling addons directory")
  #execute_process(COMMAND svn co https://robotik.informatik.uni-wuerzburg.de/slam6dprivate/trunk/addons addons
  #                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  #                OUTPUT_QUIET
  #                )
  message(STATUS "With addons.")
else()
  message(STATUS "Without addons.")
endif()

#################################################
# OPERATING SYSTEM SPECIFIC BEHAVIOUR          ##
#################################################
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS}")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ADDITIONAL_CFLAGS}")
## Special treatment for system specifics
if(APPLE)
  add_definitions(-Dfopen64=fopen)
endif()

## Multiple Cores
include(ProcessorCount)
ProcessorCount(PROCESSOR_COUNT)
if(NOT PROCESSOR_COUNT EQUAL 0)
  set(NUMBER_OF_CPUS "${PROCESSOR_COUNT}" CACHE STRING "The number of processors to use (default: ${PROCESSOR_COUNT})" )
else()
  set(NUMBER_OF_CPUS "1" CACHE STRING "The number of processors to use (default: 1)" )
endif()

# OPENMP_NUM_THREADS
if(WITH_OPENMP)
  message(STATUS "With OpenMP ")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=${NUMBER_OF_CPUS} -DOPENMP_NUM_THREADS=${NUMBER_OF_CPUS} ${OpenMP_CXX_FLAGS} -DOPENMP")
else()
  message(STATUS "Without OpenMP")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
endif()

# 3rdparty
if(WIN32)
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows/)
  link_directories(${PROJECT_SOURCE_DIR}/3rdparty/windows)
  link_directories(${BOOST_LIBRARYDIR})
  add_library(XGetopt ${PROJECT_SOURCE_DIR}/3rdparty/windows/XGetopt.cpp)
  set(CMAKE_STATIC_LIBRARY_SUFFIX "32.lib")
  # to be able to use numeric_limits<int>::max() and friends on windows
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
else()
  if (WITH_OPENGL)
    # Add include path for OpenGL without GL/-prefix
    # to avoid the include incompatibility between MACOS
    # and linux
    find_path(OPENGL_INC NAMES gl.h GL/gl.h PATHS /usr/include/GL)
    if (${OPENGL_INC})
      include_directories(${OPENGL_INC})
    else()
      message(STATUS "gl.h not found")
    endif()
  endif()
endif()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
  # using Clang
  set(ADDITIONAL_CFLAGS "-O3 -Wall -Wno-write-strings -Wno-char-subscripts -Wno-unused-result" CACHE STRING "Additional flags given to the compiler (-O3 -Wall -Wno-write-strings)" )
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
  # using GCC
  set(ADDITIONAL_CFLAGS "-O3 -Wall -finline-functions -Wno-unused-but-set-variable -Wno-write-strings -Wno-char-subscripts -Wno-unused-result" CACHE STRING "Additional flags given to the compiler (-O3 -Wall -finline-functions -Wno-write-strings)" )
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL Intel)
  # using Intel C++
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
  # using Visual Studio C++
  set(ADDITIONAL_CFLAGS "-O2" CACHE STRING "Additional flags given to the compiler ( -O2)" )
endif()

# Remove this conditional when CMake >= 3.1.0 is required.
# Older CMake is used in Debian Jessie (3.0.2) and Ubuntu Trusty (2.8.12.2).
if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
	if (${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
		# using Clang
		set(ADDITIONAL_CFLAGS "${ADDITIONAL_CFLAGS} -std=c++0x")
	elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
		# using GCC
		set(ADDITIONAL_CFLAGS "${ADDITIONAL_CFLAGS} -std=c++0x")
	endif()
else()
	# Specifying the C++ standard (-std=c++0x, -std=gnu++14, ...) by using
	# the CMAKE_CXX_FLAGS is not good cmake style (see Daniel Pfeifer in
	# https://youtu.be/bsXLMQ6WgIk).
	# By using CMAKE_CXX_STANDARD we also avoid having to implement
	# compiler specific command line arguments.
	set(CMAKE_CXX_STANDARD 11)
	# do not fall back to a previous standard
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	# enable compiler extensions like -std=gnu++11 on gcc
	# gnu extensions are needed for strdup, ftruncate, fseeko, ftello...
	set(CMAKE_CXX_EXTENSIONS ON)
endif()

# Add OpenGL includes for MACOS if needed
# The OSX OpenGL frameworks natively supports freeglut extensions
if(APPLE)
  include_directories(/System/Library/Frameworks/GLUT.framework/Headers)
  include_directories(/System/Library/Frameworks/OpenGL.framework/Headers)
endif()

# hack to "circumvent" Debug and Release folders that are created under visual studio
# this is why the INSTALL target has to be used in visual studio
if(MSVC)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/bin/Release/ CONFIGURATIONS Release DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/x64/ CONFIGURATIONS Release DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  else()
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/ CONFIGURATIONS Release DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  endif()

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/bin/Debug/ CONFIGURATIONS Debug DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/x64/ CONFIGURATIONS Debug DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  else()
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/windows/ CONFIGURATIONS Debug DESTINATION ${PROJECT_SOURCE_DIR}/windows FILES_MATCHING PATTERN "*.dll" PATTERN "*.exe")
  endif()

endif()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CFLAGS}")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ADDITIONAL_CFLAGS}")

if(APPLE)
  set(CMAKE_MACOSX_RPATH 1)
endif()

# Set include and link dirs ...
include_directories(${PROJECT_SOURCE_DIR}/include/slam6d)


#include_directories(${PROJECT_SOURCE_DIR}/3rdparty/wxthings/include/)
link_directories(${PROJECT_SOURCE_DIR}/obj)
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src/slam6d/show)
add_subdirectory(src/slam6d/slam6d)
add_subdirectory(src/slam6d/scanio)
add_subdirectory(src/slam6d/scanserver)
add_subdirectory(src/slam6d/segmentation)
add_subdirectory(src/slam6d/normals)
add_subdirectory(src/slam6d/veloslam)
add_subdirectory(src/slam6d/qtshow)
add_subdirectory(src/slam6d/grid)
add_subdirectory(src/slam6d/pmd)
add_subdirectory(src/slam6d/shapes)
add_subdirectory(src/slam6d/floorplan)
add_subdirectory(src/slam6d/thermo)
add_subdirectory(src/slam6d/slam6d/fbr)
add_subdirectory(src/slam6d/scanner)
add_subdirectory(src/slam6d/model)
add_subdirectory(src/slam6d/collision)
add_subdirectory(src/slam6d/peopleremover)
add_subdirectory(src/slam6d/spherical_quadtree)
add_subdirectory(src/slam6d/cuda)
add_subdirectory(src/slam6d/ros)
add_subdirectory(src/slam6d/tools)
add_subdirectory(src/slam6d/gps)
add_subdirectory(src/slam6d/curvefusion)
add_subdirectory(src/slam6d/mesh)
# 3rdparty must come before src/calibration because it sets
# APRILTAG_INCLUDE_DIRS
add_subdirectory(3rdparty)
add_subdirectory(src/slam6d/calibration)
#add_subdirectory(bindings)
if(WITH_ADDONS)
  message(STATUS "With 3dtk addons.")
  add_subdirectory(addons)
endif()

find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
#enable_testing()
#add_subdirectory(testing)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

####### Plugin ########

set (${PROJECT_NAME}_PLUGIN_SRCS
  src/rqt_plugin/gui_plugin.cpp
  src/mapping/mapping.cpp
  src/io/io.cpp
)

set(${PROJECT_NAME}_PLUGIN_HDRS
  include/rqt_plugin/gui_plugin.h
  include/mapping/mapping.h
  include/exceptions/bad_file_exception.h
  include/exceptions/bad_point_exception.h
  include/io/io.h
)

set(${PROJECT_NAME}_PLUGIN_UIS
  src/rqt_plugin/gui_plugin.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(${PROJECT_NAME}_PLUGIN_MOCS ${${PROJECT_NAME}_PLUGIN_HDRS})
  qt5_wrap_ui(${PROJECT_NAME}_PLUGIN_UIS_H ${${PROJECT_NAME}_PLUGIN_UIS})
else()
  qt4_wrap_cpp(${PROJECT_NAME}_PLUGIN_MOCS ${${PROJECT_NAME}_PLUGIN_HDRS})
  qt4_wrap_ui(${PROJECT_NAME}_PLUGIN_UIS_H ${${PROJECT_NAME}_PLUGIN_UIS})
endif()

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mapping.cpp
# )

add_library(${PROJECT_NAME}_plugin
  ${${PROJECT_NAME}_PLUGIN_SRCS}
  ${${PROJECT_NAME}_PLUGIN_MOCS}
  ${${PROJECT_NAME}_PLUGIN_UIS_H}
  )

add_library(${PROJECT_NAME}_my_slam6d
  src/my_slam6d/my_slam6d.cpp
  )


###### SCAN_TO_FILE #######

set(${PROJECT_NAME}_SCAN_TO_FILE_SRCS
  src/scan_to_file/scan_to_file.cpp
  src/scan_to_file/scan_to_file_node.cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mapping_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


######### PLUGIN ########

#add_executable(${PROJECT_NAME}_plugin ${${PROJECT_NAME}_PLUGIN_SRCS})

target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME}_my_slam6d scan
  ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME}_plugin Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME}_plugin ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

####### SCAN_TO_FILE ##########

add_executable(scan_to_file_node ${${PROJECT_NAME}_SCAN_TO_FILE_SRCS})

target_link_libraries(scan_to_file_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


####### SCAN_TO_FILE #######

install(TARGETS scan_to_file_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mapping.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


