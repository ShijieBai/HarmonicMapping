cmake_minimum_required(VERSION 3.16)

set(proj_name "HarmonicMapping")
project(${proj_name}
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

set(Eigen3_DIR D:/soft/eigen-prefix/share/eigen3/cmake)
set(VTK_DIR D:/soft/vtk-prefix/lib/cmake/vtk-9.2/)
set(TBB_DIR D:/One-API/tbb/latest/lib/cmake/tbb)
set(MKL_DIR D:/One-API/mkl/latest/lib/cmake/mkl)

# Use Eigen
add_library(target_interface_Eigen INTERFACE)
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(target_interface_Eigen INTERFACE Eigen3::Eigen)

# MKL
add_library(target_interface_MKL INTERFACE)
set(MKL_ARCH "intel64")
set(MKL_LINK "static")
set(MKL_ITERFACE_FULL "intel_lp64")
set(MKL_THREADING "intel_thread")
find_package(MKL CONFIG REQUIRED)
target_compile_options(target_interface_MKL INTERFACE $<TARGET_PROPERTY:MKL::MKL,INTERFACE_COMPILE_OPTIONS>)
target_include_directories(target_interface_MKL INTERFACE $<TARGET_PROPERTY:MKL::MKL,INTERFACE_INCLUDE_DIRECTORIES>)
target_link_libraries(target_interface_MKL INTERFACE $<LINK_ONLY:MKL::MKL>)

# find vtk
#find_package(VTK REQUIRED QUIET PATHS "${VTK_DIR}")
find_package(VTK REQUIRED)
if (NOT VTK_FOUND)
    message(FATAL_ERROR "${VTK_NOT_FOUND_MESSAGE}")
else ()
    message(STATUS "VTK_LIBRARIES = ${VTK_LIBRARIES}")
    message(STATUS "VTK_INCLUDE_DIRS = ${VTK_INCLUDE_DIRS}")

    add_library(target_interface_vtk INTERFACE)
    target_include_directories(target_interface_vtk INTERFACE ${VTK_INCLUDE_DIRS})
    target_link_libraries(target_interface_vtk INTERFACE ${VTK_LIBRARIES})
endif ()

# TBB
add_library(target_interface_TBB INTERFACE)
find_package(TBB CONFIG REQUIRED)
foreach(v IN LISTS TBB_IMPORTED_TARGETS)
    message(STATUS "Found TBB: ${v}")
endforeach()
target_link_libraries(target_interface_TBB INTERFACE ${TBB_IMPORTED_TARGETS})

##########################################################

add_subdirectory(src)
target_link_libraries(HarmonicMapping PUBLIC target_interface_vtk target_interface_Eigen target_interface_TBB target_interface_MKL)