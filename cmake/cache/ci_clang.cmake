include("${CMAKE_CURRENT_LIST_DIR}/__add_cache_path.cmake")

__add_cache_path(CMAKE_C_COMPILER "$ENV{GITHUB_WORKSPACE}/_deps/LLVM-16.0.1-Linux/bin/clang")
__add_cache_path(CMAKE_CXX_COMPILER "$ENV{GITHUB_WORKSPACE}/_deps/LLVM-16.0.1-Linux/bin/clang++")

__add_cache_path(CLANG_TIDY_EXE "$ENV{GITHUB_WORKSPACE}/_deps/LLVM-16.0.1-Linux/bin/clang-tidy")

set(__deps_dir "$ENV{GITHUB_WORKSPACE}/_deps/")

# Third party
__add_cache_path(nlohmann_json_DIR "${__deps_dir}/nlohmann_json/lib/cmake/nlohmann_json")
__add_cache_path(GeographicLib_DIR "${__deps_dir}/geographiclib/lib/cmake/GeographicLib")
__add_cache_path(spdlog_DIR "${__deps_dir}/spdlog/lib/cmake/spdlog")
__add_cache_path(CLI11_DIR "${__deps_dir}/cli11/lib/cmake/CLI11")
__add_cache_path(GTest_DIR "${__deps_dir}/googletest/lib/cmake/GTest")
__add_cache_path(CGAL_DIR "${__deps_dir}/cgal/lib/cmake/CGAL")

# Same Protobuf 3.0.0 version as on 'jetson'
__add_cache_path(Protobuf_ROOT "${__deps_dir}/protobuf")
set(Protobuf_USE_STATIC_LIBS ON CACHE BOOL "")

set(I_SEED_DRONE_ONBOARD_DOC ON CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_CUDA OFF CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_PEDANTIC ON CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_GIMBAL_ROTATION ON CACHE BOOL "")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "")
set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "")
