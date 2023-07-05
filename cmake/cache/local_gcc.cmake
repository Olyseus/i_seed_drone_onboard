include("${CMAKE_CURRENT_LIST_DIR}/__add_cache_path.cmake")

# https://github.com/Olyseus/dependencies/blob/42e88b7afb84a25e9d8579e873551e342faeaef2/ubuntu_22_04.rst#cuda-1211
# https://github.com/Olyseus/dependencies/blob/42e88b7afb84a25e9d8579e873551e342faeaef2/ubuntu_22_04.rst#cudnn
# https://github.com/Olyseus/dependencies/blob/42e88b7afb84a25e9d8579e873551e342faeaef2/ubuntu_22_04.rst#tensorrt
__add_cache_path(TRT_LIB_DIR "/usr/lib/x86_64-linux-gnu/")

__add_cache_path(CMAKE_C_COMPILER "/usr/bin/gcc")
__add_cache_path(CMAKE_CXX_COMPILER "/usr/bin/g++")

set(__opt_dir "$ENV{HOME}/opt")

# Third party
__add_cache_path(nlohmann_json_DIR "${__opt_dir}/nlohmann_json/lib/cmake/nlohmann_json")
__add_cache_path(GeographicLib_DIR "${__opt_dir}/geographiclib/lib/cmake/GeographicLib")
__add_cache_path(spdlog_DIR "${__opt_dir}/spdlog/lib/cmake/spdlog")
__add_cache_path(CLI11_DIR "${__opt_dir}/cli11/lib/cmake/CLI11")
__add_cache_path(GTest_DIR "${__opt_dir}/googletest/lib/cmake/GTest")
__add_cache_path(CGAL_DIR "${__opt_dir}/cgal/lib/cmake/CGAL")

# Same Protobuf 3.0.0 version as on 'jetson'
__add_cache_path(Protobuf_ROOT "$ENV{HOME}/opt/protobuf")
set(Protobuf_USE_STATIC_LIBS ON CACHE BOOL "")

set(I_SEED_DRONE_ONBOARD_DOC ON CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_CUDA ON CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_PEDANTIC OFF CACHE BOOL "")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "")
set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "")
