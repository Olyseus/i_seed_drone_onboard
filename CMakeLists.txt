cmake_minimum_required(VERSION 3.12)
project(i_seed_drone_onboard)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/module")

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
option(I_SEED_DRONE_ONBOARD_SIMULATOR "Run on simulator" OFF)

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
    json_sax_event_consumer.cpp
    json_sax_event_consumer.h
    magnetic_declination.cpp
    magnetic_declination.h
    simulator.cpp
    simulator.h
    timer.h
)

target_link_libraries(i_seed_drone_onboard_core PUBLIC protobuf::libprotobuf)

find_package(nlohmann_json CONFIG REQUIRED) # nlohmann_json::nlohmann_json
target_link_libraries(i_seed_drone_onboard_core PUBLIC nlohmann_json::nlohmann_json)

target_compile_options(
    i_seed_drone_onboard_core PUBLIC -Wall -Werror -Wpedantic -Wextra -Wfatal-errors
)

target_include_directories(
    i_seed_drone_onboard_core
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/third_party/psdk>
)

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
  target_sources(
      i_seed_drone_onboard_core
      PRIVATE
      api_code.cpp
      api_code.h
      third_party/psdk/common/osal/osal.c
      third_party/psdk/common/osal/osal.h
      third_party/psdk/common/osal/osal_fs.c
      third_party/psdk/common/osal/osal_fs.h
      third_party/psdk/common/osal/osal_socket.c
      third_party/psdk/common/osal/osal_socket.h
      drone.cpp
      drone.h
      third_party/psdk/manifold2/hal/hal_network.c
      third_party/psdk/manifold2/hal/hal_network.h
      third_party/psdk/manifold2/hal/hal_uart.c
      third_party/psdk/manifold2/hal/hal_uart.h
      third_party/psdk/manifold2/hal/hal_usb_bulk.c
      third_party/psdk/manifold2/hal/hal_usb_bulk.h
      mission_state.cpp
      mission_state.h
      server.cpp
      server.h
      third_party/psdk/application.cpp
      third_party/psdk/application.hpp
      third_party/psdk/dji_sdk_app_info.h
      third_party/psdk/dji_sdk_config.h
  )
  set_source_files_properties(
      third_party/psdk/application.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # ISO C++11 requires at least one argument for the "..." in a variadic macro
      "-Wno-pedantic;-Wno-unused-but-set-variable"
  )
  set_source_files_properties(
      server.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )
  set_source_files_properties(
      drone.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )
  set_source_files_properties(
      mission_state.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )
  set_source_files_properties(
      api_code.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )
  set_source_files_properties(
      third_party/psdk/manifold2/hal/hal_usb_bulk.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      "-Wno-pedantic"
  )
  set_source_files_properties(
      third_party/psdk/manifold2/hal/hal_network.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      "-Wno-pedantic;-Wno-unused-parameter"
  )
  set_source_files_properties(
      third_party/psdk/common/osal/osal_socket.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      # osal_socket.c:118:22: error: ordered comparison of pointer with integer zero [-Werror=extra]
      "-Wno-extra;-Wno-pedantic"
  )
  set_source_files_properties(
      third_party/psdk/manifold2/hal/hal_uart.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      "-Wno-pedantic;-Wno-unused-variable"
  )
  set_source_files_properties(
      third_party/psdk/common/osal/osal.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      "-Wno-pedantic;-Wno-unused-parameter"
  )
  set_source_files_properties(
      third_party/psdk/common/osal/osal_fs.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      "-Wno-pedantic"
  )

  # https://man7.org/linux/man-pages/man3/pthread_setname_np.3.html
  target_compile_definitions(i_seed_drone_onboard_core PUBLIC _GNU_SOURCE)

  target_compile_definitions(i_seed_drone_onboard_core PUBLIC LIBUSB_INSTALLED)

  if(I_SEED_DRONE_ONBOARD_SIMULATOR)
    target_compile_definitions(i_seed_drone_onboard_core PUBLIC I_SEED_DRONE_ONBOARD_SIMULATOR)
  endif()
endif()

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

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
  add_executable(
      i_seed_drone_onboard
      main.cpp
  )
  target_link_libraries(
      i_seed_drone_onboard PRIVATE i_seed_drone_onboard_core
  )

  add_executable(
      i_seed_drone_onboard_gimbal_heading
      main_gimbal_heading.cpp
  )
  target_link_libraries(
      i_seed_drone_onboard_gimbal_heading PRIVATE i_seed_drone_onboard_core
  )
  set_source_files_properties(
      main_gimbal_heading.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )

  add_executable(
      i_seed_drone_onboard_focal_length
      focal_length_monitor_job.cpp
      main_focal_length.cpp
  )
  target_link_libraries(
      i_seed_drone_onboard_focal_length PRIVATE i_seed_drone_onboard_core
  )
  set_source_files_properties(
      main_focal_length.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )
  set_source_files_properties(
      focal_length_monitor_job.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )

  add_executable(
      i_seed_drone_onboard_quaternion
      main_quaternion.cpp
  )
  target_link_libraries(
      i_seed_drone_onboard_quaternion PRIVATE i_seed_drone_onboard_core
  )
  set_source_files_properties(
      main_quaternion.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )

  add_executable(
      i_seed_drone_onboard_positioning
      main_positioning.cpp
  )
  target_link_libraries(
      i_seed_drone_onboard_positioning PRIVATE i_seed_drone_onboard_core
  )
  set_source_files_properties(
      main_positioning.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )
endif()

add_executable(
    i_seed_drone_onboard_core_ut
    i_seed_drone_onboard_core_ut.cpp
    magnetic_declination_ut.cpp
    main_ut.cpp
    main_ut.h
)

find_package(GTest CONFIG REQUIRED) # GTest::gtest_main
target_link_libraries(
    i_seed_drone_onboard_core_ut
    PRIVATE
    i_seed_drone_onboard_core
    GTest::gtest
)

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
  set(trt_so_dir "/usr/lib/aarch64-linux-gnu/")
  if(NOT EXISTS "${trt_so_dir}")
    message(FATAL_ERROR "${trt_so_dir} not found")
  endif()
  target_link_libraries(i_seed_drone_onboard_core PUBLIC "${trt_so_dir}/libnvinfer.so")
  target_link_libraries(i_seed_drone_onboard_core PUBLIC "${trt_so_dir}/libnvonnxparser.so")

  if("${PSDK_DIR}" STREQUAL "")
    message(FATAL_ERROR "PSDK_DIR is not set")
  endif()
  if(NOT EXISTS "${PSDK_DIR}")
    message(FATAL_ERROR "Directory not found: ${PSDK_DIR}")
  endif()

  target_include_directories(
      i_seed_drone_onboard_core
      PUBLIC
      "${PSDK_DIR}/include"
      "/usr/include/libusb-1.0/"
  )

  find_package(Threads REQUIRED) # Threads::Threads
  find_package(FFMPEG REQUIRED) # FFMPEG_LIBRARIES

  target_link_libraries(
      i_seed_drone_onboard_core
      PUBLIC
      "${PSDK_DIR}/lib/libpayloadsdk.a"
      "${FFMPEG_LIBRARIES}"
      Threads::Threads
      "-lusb-1.0"
  )
  target_compile_definitions(i_seed_drone_onboard_core PUBLIC ADVANCED_SENSING=1)
else()
  if("${TRT_LIB_DIR}" STREQUAL "")
    message(FATAL_ERROR "TRT_LIB_DIR not set")
  endif()
  if(NOT EXISTS "${TRT_LIB_DIR}")
    message(FATAL_ERROR "Directory not found: ${TRT_LIB_DIR}")
  endif()

  target_include_directories(i_seed_drone_onboard_core PUBLIC "${TRT_LIB_DIR}/include")
  target_link_libraries(i_seed_drone_onboard_core PUBLIC "${TRT_LIB_DIR}/lib/libnvinfer.so")
  target_link_libraries(i_seed_drone_onboard_core PUBLIC "${TRT_LIB_DIR}/lib/libnvonnxparser.so")
endif()

if(I_SEED_DRONE_ONBOARD_PEDANTIC)
  # Disable warnings in automatically generated files
  file(
      COPY_FILE
      "${CMAKE_CURRENT_LIST_DIR}/clang-tidy-disabled"
      "${CMAKE_CURRENT_BINARY_DIR}/.clang-tidy"
  )

  if(I_SEED_DRONE_ONBOARD_MANIFOLD)
    set_target_properties(
        i_seed_drone_onboard
        PROPERTIES
        CXX_CLANG_TIDY clang-tidy
    )
  endif()

  set_target_properties(
      i_seed_drone_onboard_core
      PROPERTIES
      CXX_CLANG_TIDY clang-tidy
  )
endif()

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
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
endif()

enable_testing()
add_test(
    NAME i_seed_drone_onboard_core_ut
    COMMAND i_seed_drone_onboard_core_ut
    "${CMAKE_CURRENT_LIST_DIR}"
)
