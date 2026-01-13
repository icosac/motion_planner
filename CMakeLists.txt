cmake_minimum_required(VERSION 3.22)

project(motion_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(motion_planner STATIC
    src/rrt.cpp
    src/rrt_star.cpp
    src/mpdp_reeds_shepp.cpp
)

set(COMPILE_CXX_EXEC OFF CACHE BOOL "" FORCE)
set(COMPILE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(COMPILE_TEST OFF CACHE BOOL "" FORCE)
set(COMPILE_TESTS OFF CACHE BOOL "" FORCE)
set(CUDA_ON OFF CACHE BOOL "" FORCE)
set(COMPILE_CUDA_EXEC OFF CACHE BOOL "" FORCE)
set(MPDP_BUILD_POLYNOMIALS OFF CACHE BOOL "" FORCE)

add_subdirectory(external/mpdp EXCLUDE_FROM_ALL)

target_include_directories(motion_planner
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(motion_planner PUBLIC cxx_std_17)
target_link_libraries(motion_planner PUBLIC MPDPCC)

enable_testing()

add_executable(rrt_reeds_shepp_example examples/rrt_reeds_shepp_example.cpp)
target_link_libraries(rrt_reeds_shepp_example PRIVATE motion_planner)

# Documentation target (runs only when explicitly built).
find_package(Doxygen QUIET)
set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/docs")
set(DOXYGEN_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/examples")
set(DOXYGEN_EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/external ${CMAKE_CURRENT_SOURCE_DIR}/build")
set(DOXYGEN_IN "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")
set(DOXYGEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${CMAKE_COMMAND} -E make_directory "${DOXYGEN_OUTPUT_DIR}"
        COMMAND ${DOXYGEN_EXECUTABLE} "${DOXYGEN_OUT}"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
else()
    add_custom_target(docs
        COMMAND ${CMAKE_COMMAND} -E echo "Doxygen not found. Install Doxygen to build docs."
        COMMAND ${CMAKE_COMMAND} -E false
        COMMENT "Documentation target requires Doxygen"
        VERBATIM
    )
endif()
