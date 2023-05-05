cmake_minimum_required(VERSION 3.12)

# Set system name, this sets the variable CMAKE_CROSSCOMPILING
set(CMAKE_SYSTEM_NAME "Linux")

set(CROSS_COMPILE_TOOLCHAIN_PREFIX "aarch64-linux-gnu")

set(__gcc_name "${CROSS_COMPILE_TOOLCHAIN_PREFIX}-gcc")
find_program(__gcc_location "${__gcc_name}")
if(NOT __gcc_location)
  message(
      FATAL_ERROR
      "GCC not found: ${__gcc_name} (update PATH environment variable)"
  )
endif()

get_filename_component(
    CROSS_COMPILE_TOOLCHAIN_PATH "${__gcc_location}" DIRECTORY
)
if("${CROSS_COMPILE_TOOLCHAIN_PATH}" STREQUAL "")
  message(FATAL_ERROR "CROSS_COMPILE_TOOLCHAIN_PATH is not set")
endif()

get_filename_component(
    CMAKE_SYSROOT
    "${CROSS_COMPILE_TOOLCHAIN_PATH}/.."
    ABSOLUTE
)
if(NOT IS_DIRECTORY "${CMAKE_SYSROOT}")
  message(FATAL_ERROR "Directory not found: ${CMAKE_SYSROOT}")
endif()

set(TOOLCHAIN_PATH_AND_PREFIX ${CROSS_COMPILE_TOOLCHAIN_PATH}/${CROSS_COMPILE_TOOLCHAIN_PREFIX})

foreach(__x "/usr/lib/aarch64-linux-gnu" "/usr/local/cuda-10.2/targets/aarch64-linux/lib" "/aarch64-linux-gnu/lib64")
  set(__dir_path "${CMAKE_SYSROOT}/${__x}")
  if(NOT IS_DIRECTORY "${__dir_path}")
    message(FATAL_ERROR "Directory not found: ${__dir_path}")
  endif()
  set(
      CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${__dir_path}"
  )
endforeach()

set(CMAKE_C_COMPILER     "${TOOLCHAIN_PATH_AND_PREFIX}-gcc"     CACHE PATH "C compiler" )
set(CMAKE_CXX_COMPILER   "${TOOLCHAIN_PATH_AND_PREFIX}-g++"     CACHE PATH "C++ compiler" )
set(CMAKE_ASM_COMPILER   "${TOOLCHAIN_PATH_AND_PREFIX}-as"      CACHE PATH "Assembler" )
set(CMAKE_C_PREPROCESSOR "${TOOLCHAIN_PATH_AND_PREFIX}-cpp"     CACHE PATH "Preprocessor" )
set(CMAKE_STRIP          "${TOOLCHAIN_PATH_AND_PREFIX}-strip"   CACHE PATH "strip" )

if(EXISTS "${TOOLCHAIN_PATH_AND_PREFIX}-gcc-ar")
  # Prefer gcc-ar over binutils ar: https://gcc.gnu.org/wiki/LinkTimeOptimizationFAQ
  set(CMAKE_AR           "${TOOLCHAIN_PATH_AND_PREFIX}-gcc-ar"  CACHE PATH "Archiver" )
else()
  set(CMAKE_AR           "${TOOLCHAIN_PATH_AND_PREFIX}-ar"      CACHE PATH "Archiver" )
endif()

set(CMAKE_LINKER         "${TOOLCHAIN_PATH_AND_PREFIX}-ld"      CACHE PATH "Linker" )
set(CMAKE_NM             "${TOOLCHAIN_PATH_AND_PREFIX}-nm"      CACHE PATH "nm" )
set(CMAKE_OBJCOPY        "${TOOLCHAIN_PATH_AND_PREFIX}-objcopy" CACHE PATH "objcopy" )
set(CMAKE_OBJDUMP        "${TOOLCHAIN_PATH_AND_PREFIX}-objdump" CACHE PATH "objdump" )
set(CMAKE_RANLIB         "${TOOLCHAIN_PATH_AND_PREFIX}-ranlib"  CACHE PATH "ranlib" )
set(CMAKE_RC_COMPILER    "${TOOLCHAIN_PATH_AND_PREFIX}-windres" CACHE PATH "WindowsRC" )
set(CMAKE_Fortran_COMPILER "${TOOLCHAIN_PATH_AND_PREFIX}-gfortran" CACHE PATH "gfortran" )

set(CMAKE_SYSTEM_PROCESSOR "aarch64")
