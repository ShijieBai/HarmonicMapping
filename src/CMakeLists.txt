file(GLOB src_list *.cpp include/*.h)

set(include_dir "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(HarmonicMapping ${src_list})

target_include_directories(HarmonicMapping PUBLIC ${include_dir})