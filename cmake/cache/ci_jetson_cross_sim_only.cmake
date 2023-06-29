include("${CMAKE_CURRENT_LIST_DIR}/__add_cache_path.cmake")

__add_cache_path(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../jetson_toolchain.cmake")

set(__deps_dir "$ENV{GITHUB_WORKSPACE}/_deps/")
set(__jetson_cross "${__deps_dir}/jetson_cross")

__add_cache_path(CUDAToolkit_ROOT "${__jetson_cross}/usr/local/cuda-10.2/bin")
__add_cache_path(TRT_SO_DIR "${__jetson_cross}/usr/lib/aarch64-linux-gnu")
__add_cache_path(PSDK_DIR "${__jetson_cross}/usr")

# Protobuf compiler is run on host machine, so it should be x64
__add_cache_path(Protobuf_PROTOC_EXECUTABLE "${__deps_dir}/protobuf/bin/protoc")

set(I_SEED_DRONE_ONBOARD_DOC OFF CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_CUDA ON CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_INTERCONNECTION OFF CACHE BOOL "")
set(I_SEED_DRONE_ONBOARD_SIMULATOR ON CACHE BOOL "")

set(CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "")
