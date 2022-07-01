cmake_minimum_required(VERSION 3.12)
project(i_seed_drone_onboard)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "C++ standard required")
set(CMAKE_CXX_EXTENSIONS FALSE CACHE BOOL "C++ standard extension")

if(NOT "${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
  message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is not supported")
endif()

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
  set(I_SEED_DRONE_ONBOARD_MANIFOLD FALSE)
elseif("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
  set(I_SEED_DRONE_ONBOARD_MANIFOLD TRUE)
else()
  message(FATAL_ERROR "${CMAKE_SYSTEM_PROCESSOR} is not supported")
endif()

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
  set(acm "/dev/ttyACM0")
  if(NOT EXISTS "${acm}")
    message(WARNING "${acm} not present, drone is not connected")
  endif()
endif()

option(I_SEED_DRONE_ONBOARD_PEDANTIC "Enable pedantic checks" OFF)

# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

find_package(Protobuf REQUIRED) # protobuf::protoc

set(input_proto "${CMAKE_CURRENT_LIST_DIR}/interconnection.proto")
set(gen_h "${CMAKE_CURRENT_BINARY_DIR}/interconnection.pb.h")
set(gen_cc "${CMAKE_CURRENT_BINARY_DIR}/interconnection.pb.cc")
set(gen_java_dir "${CMAKE_CURRENT_LIST_DIR}/java")
set(gen_java "${gen_java_dir}/interconnection/Interconnection.java")

add_custom_command(
    OUTPUT "${gen_h}" "${gen_cc}" "${gen_java}"
    COMMAND
        protobuf::protoc
    ARGS
        "--proto_path=${CMAKE_CURRENT_LIST_DIR}"
        "--cpp_out=${CMAKE_CURRENT_BINARY_DIR}"
        "--java_out=${gen_java_dir}"
        "${input_proto}"
    DEPENDS "${input_proto}" protobuf::protoc
    COMMENT "Generate C++ and Java code from ${input_proto}"
    VERBATIM
)

set_source_files_properties(${gen_h} ${gen_cc} PROPERTIES GENERATED TRUE)

set_source_files_properties(
    ${gen_cc}
    PROPERTIES
    COMPILE_OPTIONS
    "-Wno-unused-parameter"
)

add_library(
    i_seed_drone_onboard_core
    ${gen_cc}
    ${gen_h}
    boost_assert.cpp
)

find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(i_seed_drone_onboard_core PUBLIC Boost::filesystem)

find_package(spdlog CONFIG REQUIRED) # spdlog::spdlog
target_link_libraries(
    i_seed_drone_onboard_core
    PUBLIC
    spdlog::spdlog
)

target_compile_definitions(
    i_seed_drone_onboard_core PUBLIC BOOST_ENABLE_ASSERT_HANDLER
)

add_executable(
    i_seed_drone_onboard
    main.cpp
)

target_link_libraries(
    i_seed_drone_onboard PRIVATE i_seed_drone_onboard_core
)

add_executable(
    i_seed_drone_onboard_core_ut
    i_seed_drone_onboard_core_ut.cpp
)

find_package(GTest CONFIG REQUIRED) # GTest::gtest_main
target_link_libraries(
    i_seed_drone_onboard_core_ut
    PRIVATE
    i_seed_drone_onboard_core
    GTest::gtest_main
)

if(I_SEED_DRONE_ONBOARD_PEDANTIC)
  # Disable warnings in automatically generated files
  file(
      COPY_FILE
      "${CMAKE_CURRENT_LIST_DIR}/clang-tidy-disabled"
      "${CMAKE_CURRENT_BINARY_DIR}/.clang-tidy"
  )

  target_compile_options(
      i_seed_drone_onboard_core PUBLIC -Wall -Werror -Wpedantic -Wextra
  )

  set_target_properties(
      i_seed_drone_onboard
      PROPERTIES
      CXX_CLANG_TIDY clang-tidy
  )

  set_target_properties(
      i_seed_drone_onboard_core
      PROPERTIES
      CXX_CLANG_TIDY clang-tidy
  )
endif()

find_package(CLI11 CONFIG REQUIRED) # CLI11::CLI11
target_link_libraries(
    i_seed_drone_onboard
    PRIVATE
    CLI11::CLI11
)

install(
    TARGETS i_seed_drone_onboard
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

enable_testing()
add_test(
    NAME i_seed_drone_onboard_core_ut
    COMMAND i_seed_drone_onboard_core_ut
    "${CMAKE_CURRENT_LIST_DIR}"
)
