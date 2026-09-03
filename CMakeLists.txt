# https://pybind11.readthedocs.io/en/stable/compiling.html
cmake_minimum_required(VERSION 3.15...3.29)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(marinholab_solver_qpoases LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build the standalone C++ example (example/)." OFF)

find_package(Eigen3 REQUIRED)

# https://stackoverflow.com/questions/38296756/what-is-the-idiomatic-way-in-cmake-to-add-the-fpic-compiler-option
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
# See subdirectory for the flag below
set(QPOASES_BUILD_EXAMPLES FALSE)
add_subdirectory(qpOASES)

# Static library with the Solver/Configuration implementation. Both the
# pybind11 module and the standalone example link against it, so the C++
# code is compiled exactly once.
add_library(marinholab_qpoases STATIC
        src/core_function.cpp
    )

target_include_directories(marinholab_qpoases
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        qpOASES/include
    )

target_link_libraries(marinholab_qpoases PUBLIC Eigen3::Eigen qpOASES)

# Build the standalone C++ example (see example/example.cpp).
if (BUILD_EXAMPLES)
    add_executable(example_qpoases example/example.cpp)
    target_link_libraries(example_qpoases PRIVATE marinholab_qpoases)
endif ()

# Python extension.
set(PYBIND11_FINDPYTHON ON)
add_subdirectory(pybind11)
pybind11_add_module(_core MODULE
        src/core.cpp
    )

target_link_libraries(_core PRIVATE marinholab_qpoases)
# Version is dynamically obtained. See pyproject.toml
# Activating this causes issues with Windows compilation.
# target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})