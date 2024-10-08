###########################################################################
# CMakeLists.txt generated for component lqr_k
# Product type: STATIC library
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(lqr_k)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# Define common variables that are used within the whole project.
set(SYSLIB_PREFIX $<IF:$<AND:$<BOOL:${WIN32}>,$<NOT:$<BOOL:${MINGW}>>>,lib,>)

###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR ../../.. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT D:/Matlab2023b CACHE PATH "")

# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# Definition of target "lqr_k".
add_library(lqr_k STATIC ${START_DIR}/codegen/lib/lqr_k/rt_nonfinite.c
    ${START_DIR}/codegen/lib/lqr_k/rtGetNaN.c
    ${START_DIR}/codegen/lib/lqr_k/rtGetInf.c
    ${START_DIR}/codegen/lib/lqr_k/lqr_k_initialize.c
    ${START_DIR}/codegen/lib/lqr_k/lqr_k_terminate.c
    ${START_DIR}/codegen/lib/lqr_k/lqr_k.c)

# Set properties for target "lqr_k".
set_target_properties(lqr_k PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/lqr_k/$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/lqr_k/$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/lqr_k/$<0:>")

# Specify language features required for target "lqr_k".
target_compile_features(lqr_k PUBLIC c_std_99)

# Specify compiler preprocessor definitions for target "lqr_k".
target_compile_definitions(lqr_k PRIVATE -DMODEL=lqr_k)

# Specify include directories for target "lqr_k".
target_include_directories(lqr_k PUBLIC $<BUILD_INTERFACE:${START_DIR}/codegen/lib/lqr_k>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/codegen/lib/lqr_k>
    $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>)

# Specify library link dependencies for target "lqr_k".  CMake generator 
# expressions are used to create a CMakeLists.txt file that supports 
# multiple platforms with differently named system library dependencies.
target_link_libraries(lqr_k INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET lqr_k POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created STATIC library: $<TARGET_FILE:lqr_k>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(lqr_k)
endif()
