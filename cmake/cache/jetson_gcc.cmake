include("${CMAKE_CURRENT_LIST_DIR}/__add_cache_path.cmake")

__add_cache_path(CMAKE_C_COMPILER "/usr/bin/gcc")
__add_cache_path(CMAKE_CXX_COMPILER "/usr/bin/g++")

set(__opt_dir "$ENV{HOME}/opt")

# Third party
__add_cache_path(nlohmann_json_DIR "${__opt_dir}/nlohmann_json/lib/cmake/nlohmann_json")
__add_cache_path(GeographicLib_DIR "${__opt_dir}/geographiclib/lib/cmake/GeographicLib")
__add_cache_path(spdlog_DIR "${__opt_dir}/spdlog/lib/cmake/spdlog")
__add_cache_path(CLI11_DIR "${__opt_dir}/cli11/lib/cmake/CLI11")
__add_cache_path(GTest_DIR "${__opt_dir}/googletest/lib/cmake/GTest")
__add_cache_path(PSDK_DIR "${__opt_dir}/psdk")

set(I_SEED_DRONE_ONBOARD_DOC OFF CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_CUDA ON CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_PEDANTIC OFF CACHE BOOL "")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "")
set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "")
