# https://pybind11.readthedocs.io/en/stable/compiling.html
cmake_minimum_required(VERSION 3.15...3.29)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

include(cmake/GetVersion.cmake)
project(marinholab_solver_qpoases VERSION ${QPOASES_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build the standalone C++ example (example/)." OFF)
# The pybind11 Python extension is built via `pip install .` / setup.py,
# which passes -DBUILD_PYTHON=ON; the C++-only build (Debian package,
# find_package consumers) leaves it OFF.
option(BUILD_PYTHON "Build the pybind11 Python extension module (_core)." OFF)
# Shared library by default; build a static library with -DBUILD_SHARED_LIBS=OFF.
# The option() establishes the default while still honouring a value that a
# parent project (or the command line) provides.
option(BUILD_SHARED_LIBS "Build the marinholab_qpoases shared library" ON)

find_package(Eigen3 REQUIRED)

# https://stackoverflow.com/questions/38296756/what-is-the-idiomatic-way-in-cmake-to-add-the-fpic-compiler-option
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# The Python build bundles the static library into the pybind11 module, so
# qpOASES must be static there. The C++-only build follows BUILD_SHARED_LIBS
# (shared by default).
if(BUILD_PYTHON)
    set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
endif()
# See subdirectory for the flag below
set(QPOASES_BUILD_EXAMPLES FALSE)
add_subdirectory(qpOASES)

# The Solver/Configuration implementation. Both the pybind11 module and the
# standalone example link against it, so the C++ code is compiled exactly once.
# Shared by default; static with -DBUILD_SHARED_LIBS=OFF.
add_library(marinholab_qpoases
        src/core_function.cpp
    )
add_library(marinholab::solvers::qpoases ALIAS marinholab_qpoases)

set_target_properties(marinholab_qpoases PROPERTIES
    OUTPUT_NAME marinholab_solver_qpoases
    POSITION_INDEPENDENT_CODE ON
    VERSION ${PROJECT_VERSION}
    SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

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

# Python extension (built only for `pip install .` / setup.py).
if(BUILD_PYTHON)
    set(PYBIND11_FINDPYTHON ON)
    add_subdirectory(pybind11)
    pybind11_add_module(_core MODULE
            src/core.cpp
        )
    target_link_libraries(_core PRIVATE marinholab_qpoases)
    # Version is dynamically obtained. See pyproject.toml
    # Activating this causes issues with Windows compilation.
    # target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})
endif()

# ---- Install / CMake package (C++ part only) ----
# Runs only under `cmake --install` (e.g. from the Debian package build); the
# `pip install .` path uses build_ext and never invokes the install rules.
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Values passed to the package config as @PACKAGE_<VAR>@ (see PATH_VARS below).
# Using the actual install dirs keeps the config correct on multiarch Debian
# (libdir = lib/<arch>, includedir = include).
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})

install(TARGETS marinholab_qpoases
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(BUILD_EXAMPLES)
    install(TARGETS example_qpoases RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/marinholab_solver_qpoases-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/marinholab_solver_qpoases-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/marinholab_solver_qpoases
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/marinholab_solver_qpoases-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/marinholab_solver_qpoases-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/marinholab_solver_qpoases-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/marinholab_solver_qpoases)
