cmake_minimum_required(VERSION 3.10)
project(SolverDevelopment)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# MATLAB root directory
# set(MATLAB_ROOT "/usr/local/MATLAB/R2024a") # Replace with your MATLAB installation path
# Use only Homebrew for pkg-config (prioritize it over /usr/local)
set(ENV{PKG_CONFIG_PATH} "/opt/homebrew/lib/pkgconfig:/opt/homebrew/share/pkgconfig")

# Find packages
find_package(PkgConfig REQUIRED)
# CPPAD output the .pc file, find it with pkgconfig
# CPPADCG is header file only, if you install it default, you don't need to find it explicitly
pkg_check_modules(CPPAD REQUIRED cppad)
find_package(Eigen3 REQUIRED)

# Boost 1.89.0 on macOS doesn't provide component config files, so we manually set the paths
set(Boost_INCLUDE_DIRS "/opt/homebrew/include")
set(Boost_LIBRARY_DIRS "/opt/homebrew/lib")
set(Boost_LIBRARIES "/opt/homebrew/lib/libboost_filesystem.dylib")
set(Boost_FOUND TRUE)

find_package(yaml-cpp REQUIRED) # Yaml for reading the hyper-parameters for the solver
find_package(piqp REQUIRED) # piqp is header only

# Add library directories for linking (Homebrew for most deps, /usr/local for manually installed PIQP)
link_directories(/opt/homebrew/lib /usr/local/lib)

# Set RPATH so executables can find shared libraries at runtime
# Note: PIQP is in /usr/local/lib, other deps in /opt/homebrew/lib
set(CMAKE_INSTALL_RPATH "/opt/homebrew/lib;/usr/local/lib")
set(CMAKE_BUILD_RPATH "/opt/homebrew/lib;/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# print if find eigen and boost, and pkgconfig
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
message(STATUS "CPPAD include dir: ${CPPAD_INCLUDE_DIRS}")
message(STATUS "YAML-CPP include dir: ${YAML_CPP_INCLUDE_DIR}")


# Include MATLAB headers
# include_directories(${MATLAB_ROOT}/extern/include)
# link_directories(${MATLAB_ROOT}/bin/glnxa64)
# Add subdirectories
add_subdirectory(core)
add_subdirectory(examples)