cmake_minimum_required(VERSION 3.12)

project(i_seed_drone_onboard VERSION 0.0.9)

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
  set(LINUX_UART_DEV1 "/dev/ttyS0")
  if(NOT EXISTS "${LINUX_UART_DEV1}")
    message(WARNING "${LINUX_UART_DEV1} not present, drone is not connected")
  endif()

  set(LINUX_UART_DEV2 "/dev/ttyACM0")
  if(NOT EXISTS "${LINUX_UART_DEV2}")
    message(WARNING "${LINUX_UART_DEV2} not present, drone is not connected")
  endif()
endif()

option(I_SEED_DRONE_ONBOARD_PEDANTIC "Enable pedantic checks" OFF)
option(I_SEED_DRONE_ONBOARD_INTERCONNECTION "Establish interconnection with RC" ON)
option(I_SEED_DRONE_ONBOARD_SIMULATOR "Run on simulator" OFF)
option(I_SEED_DRONE_ONBOARD_DOC "Build Doxygen documentation" OFF)
option(I_SEED_DRONE_ONBOARD_CUDA "Build with CUDA" ON)
option(I_SEED_DRONE_ONBOARD_GIMBAL_ROTATION "Enable gimbal rotation" ON)

if(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  if(I_SEED_DRONE_ONBOARD_SIMULATOR)
    # RC: connected
    # Simulator: ON
    # RTK: OFF
    # Laser: simulated values, no LASER_RANGE packets
    message("Configuration: Simulator flight with connected RC")
  else()
    # RC: connected
    # Simulator: OFF
    # RTK: ON
    # Laser: real values
    message("Configuration: REAL FLIGHT")
    if(NOT I_SEED_DRONE_ONBOARD_GIMBAL_ROTATION)
      message(FATAL_ERROR "Gimbal rotation should be enabled in real flight")
    endif()
  endif()
else()
  if(I_SEED_DRONE_ONBOARD_SIMULATOR)
    # RC: OFF
    # Simulator: ON
    # RTK: OFF
    # Laser: simulated values, no LASER_RANGE packets
    message("Configuration: Simulator flight without RC")
  else()
    message(FATAL_ERROR "Configuration: INVALID")
  endif()
endif()

if(I_SEED_DRONE_ONBOARD_DOC)
  # Trailing return type issue: https://github.com/doxygen/doxygen/issues/5492
  find_package(Doxygen REQUIRED dot)

  set(DOXYGEN_PROJECT_NAME "I-Seed drone's onboard service")
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_WARN_AS_ERROR YES)
  set(DOXYGEN_SOURCE_BROWSER NO)
  set(DOXYGEN_VERBATIM_HEADERS NO)
  set(DOXYGEN_SHOW_FILES NO)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_SORT_MEMBER_DOCS NO)
  set(DOXYGEN_FILE_PATTERNS *.cpp *.h)
  set(DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_LIST_DIR}/docs")

  set(DOXYGEN_EXCLUDE_PATTERNS */_deps/* */_builds/* */third_party/* *_ut.cpp)

  # To generate PDF enter the binary directory 'latex/' and run 'make'
  set(DOXYGEN_GENERATE_HTML YES)
  set(DOXYGEN_GENERATE_LATEX YES)

  set(
      DOXYGEN_EXCLUDE
      battery_monitor_main.cpp
      convert_main.cpp
      inference_main.cpp
      main_camera_test.cpp
      main_focal_length.cpp
      main_geoid.cpp
      main_gimbal_heading.cpp
      main_positioning.cpp
      main_quaternion.cpp
      main_sdcard_clean.cpp
      main_sdcard_list.cpp
      main_ut.h
      olyseus_verify.h
      simulator.cpp
      simulator.h
  )

  doxygen_add_docs(
      doxygen_doc
      ALL
      ${CMAKE_CURRENT_LIST_DIR}
      COMMENT "Generate doxygen documentation"
  )
endif()

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
    bounding_box.cpp
    bounding_box.h
    condition_flag.cpp
    condition_flag.h
    converter.cpp
    converter.h
    detection_result.h
    home_altitude.cpp
    home_altitude.h
    laser_range.cpp
    laser_range.h
    lat_lon.h
    mission_aligned_cell.cpp
    mission_aligned_cell.h
    mission_builder.cpp
    mission_builder.h
    mission_cell.cpp
    mission_cell.h
    mission_directed_cell.cpp
    mission_directed_cell.h
    mission_directed_polygon.cpp
    mission_directed_polygon.h
    mission_polygon.cpp
    mission_polygon.h
    mission_simple_polygon.cpp
    mission_simple_polygon.h
    olyseus_verify.h
    polygon_slicer.cpp
    polygon_slicer.h
    timer.h
    utils.cpp
    utils.h
    waypoint.cpp
    waypoint.h
)

find_package(CGAL CONFIG REQUIRED)
target_link_libraries(i_seed_drone_onboard_core PUBLIC CGAL::CGAL)

if(I_SEED_DRONE_ONBOARD_CUDA)
  target_sources(
      i_seed_drone_onboard_core
      PRIVATE
      inference.cpp
      inference.h
  )

  set_source_files_properties(
      inference.cpp
      PROPERTIES
      COMPILE_OPTIONS
      "-Wno-deprecated-declarations;-Wno-unused-parameter"
  )
endif()

set_source_files_properties(
    utils.cpp
    PROPERTIES
    COMPILE_OPTIONS
    "-Wno-deprecated-declarations;-Wno-unused-parameter"
)

find_package(OpenCV CONFIG REQUIRED)
target_link_libraries(
    i_seed_drone_onboard_core
    PUBLIC
    opencv_core
    opencv_highgui
    opencv_imgcodecs
    opencv_imgproc
)

if(I_SEED_DRONE_ONBOARD_CUDA)
  find_package(CUDAToolkit REQUIRED) # CUDA::cudart
  target_link_libraries(i_seed_drone_onboard_core PUBLIC CUDA::cudart)
endif()

target_link_libraries(i_seed_drone_onboard_core PUBLIC protobuf::libprotobuf)

find_package(nlohmann_json CONFIG REQUIRED) # nlohmann_json::nlohmann_json
target_link_libraries(i_seed_drone_onboard_core PUBLIC nlohmann_json::nlohmann_json)

find_package(JPEG REQUIRED)
target_link_libraries(i_seed_drone_onboard_core PUBLIC JPEG::JPEG)

find_package(GeographicLib CONFIG REQUIRED)
target_link_libraries(i_seed_drone_onboard_core PUBLIC ${GeographicLib_LIBRARIES})

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(i_seed_drone_onboard_core PUBLIC Eigen3::Eigen)

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
      channel.cpp
      channel.h
      mission_state.cpp
      mission_state.h
      server.cpp
      server.h
      third_party/psdk/application.cpp
      third_party/psdk/application.hpp
      third_party/psdk/dji_config_manager.c
      third_party/psdk/dji_config_manager.h
      third_party/psdk/dji_sdk_app_info.h
      third_party/psdk/dji_sdk_config.h
      camera.cpp
      camera.h
      simulator.cpp
      simulator.h
      mission.h
      mission.cpp
  )

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set_source_files_properties(
        camera.cpp
        PROPERTIES
        COMPILE_OPTIONS
        "-Wno-nested-anon-types"
    )

    set_source_files_properties(
        simulator.cpp
        PROPERTIES
        COMPILE_OPTIONS
        "-Wno-nested-anon-types"
    )
  endif()

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set_source_files_properties(
        third_party/psdk/application.cpp
        PROPERTIES
        COMPILE_OPTIONS
        # ISO C++11 requires at least one argument for the "..." in a variadic macro
        "-Wno-pedantic;-Wno-unused-but-set-variable"
    )
  else()
    set_source_files_properties(
        third_party/psdk/application.cpp
        PROPERTIES
        COMPILE_OPTIONS
        # ISO C++11 requires at least one argument for the "..." in a variadic macro
        "-Wno-pedantic"
    )
  endif()

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
      third_party/psdk/dji_config_manager.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      "-Wno-pedantic"
  )
  set_source_files_properties(
      third_party/psdk/manifold2/hal/hal_usb_bulk.c
      PROPERTIES
      COMPILE_OPTIONS
      # dji_error.h:44:1: error: ISO C restricts enumerator values to range of ‘int’ [-Werror=pedantic]
      # directive output may be truncated writing 4 bytes into a region of size between 1 and 64
      "-Wno-pedantic;-Wno-format-truncation"
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
      "-Wno-extra;-Wno-pedantic;-Wno-unused-result"
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

  if(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
    target_compile_definitions(i_seed_drone_onboard_core PUBLIC I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  endif()

  if(I_SEED_DRONE_ONBOARD_GIMBAL_ROTATION)
    target_compile_definitions(i_seed_drone_onboard_core PUBLIC I_SEED_DRONE_ONBOARD_GIMBAL_ROTATION)
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

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
  add_executable(
      i_seed_drone_onboard
      main.cpp
  )

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set_source_files_properties(
        main.cpp
        PROPERTIES
        COMPILE_OPTIONS
        "-Wno-nested-anon-types"
    )
  endif()

  add_executable(
      i_seed_drone_camera_test
      main_camera_test.cpp
  )

  set_source_files_properties(
      main_camera_test.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )

  add_executable(
      i_seed_drone_sdcard_list
      main_sdcard_list.cpp
  )

  add_executable(
      i_seed_drone_sdcard_clean
      main_sdcard_clean.cpp
  )

  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set_source_files_properties(
        main_sdcard_clean.cpp
        PROPERTIES
        COMPILE_OPTIONS
        "-Wno-nested-anon-types"
    )

    set_source_files_properties(
        main_sdcard_list.cpp
        PROPERTIES
        COMPILE_OPTIONS
        "-Wno-nested-anon-types"
    )
  endif()

  target_link_libraries(
      i_seed_drone_sdcard_clean PRIVATE i_seed_drone_onboard_core
  )

  target_link_libraries(
      i_seed_drone_sdcard_list PRIVATE i_seed_drone_onboard_core
  )

  target_link_libraries(
      i_seed_drone_camera_test PRIVATE i_seed_drone_onboard_core
  )

  add_executable(
      i_seed_battery_monitor
      battery_monitor_main.cpp
  )

  set_source_files_properties(
      battery_monitor_main.cpp
      PROPERTIES
      COMPILE_OPTIONS
      # libusb.h:736:4: error: ISO C++ forbids zero-size array ‘dev_capability_data’ [-Werror=pedantic]
      "-Wno-ignored-qualifiers;-Wno-pedantic"
  )

  target_link_libraries(
      i_seed_battery_monitor PRIVATE i_seed_drone_onboard_core
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
    converter_ut.cpp
    home_altitude_ut.cpp
    i_seed_drone_onboard_core_ut.cpp
    main_ut.cpp
    main_ut.h
    mission_aligned_cell_ut.cpp
    mission_cell_ut.cpp
    mission_directed_cell_ut.cpp
    mission_directed_polygon_ut.cpp
    mission_polygon_ut.cpp
    mission_simple_polygon_ut.cpp
    polygon_slicer_ut.cpp
    utils_ut.cpp
)

find_package(CLI11 CONFIG REQUIRED) # CLI11::CLI11

add_executable(i_seed_geoid main_geoid.cpp)
target_link_libraries(
    i_seed_geoid PRIVATE i_seed_drone_onboard_core CLI11::CLI11
)

if(I_SEED_DRONE_ONBOARD_CUDA)
  add_executable(i_seed_convert convert_main.cpp)
  target_link_libraries(i_seed_convert PRIVATE i_seed_drone_onboard_core)

  set_source_files_properties(
      convert_main.cpp
      PROPERTIES
      COMPILE_OPTIONS
      "-Wno-deprecated-declarations;-Wno-unused-parameter"
  )

  target_link_libraries(i_seed_convert PUBLIC CLI11::CLI11)

  add_executable(i_seed_inference inference_main.cpp)
  target_link_libraries(i_seed_inference PRIVATE i_seed_drone_onboard_core)

  set_source_files_properties(
      inference_main.cpp
      PROPERTIES
      COMPILE_OPTIONS
      "-Wno-deprecated-declarations;-Wno-unused-parameter"
  )

  target_link_libraries(i_seed_inference PUBLIC CLI11::CLI11)
endif()

find_package(GTest CONFIG REQUIRED) # GTest::gtest_main
target_link_libraries(
    i_seed_drone_onboard_core_ut
    PRIVATE
    i_seed_drone_onboard_core
    GTest::gtest
)

if(I_SEED_DRONE_ONBOARD_CUDA)
  if("${TRT_SO_DIR}" STREQUAL "")
    if(I_SEED_DRONE_ONBOARD_MANIFOLD)
      set(TRT_SO_DIR "/usr/lib/aarch64-linux-gnu/")
    else()
      set(TRT_SO_DIR "/usr/lib/x86_64-linux-gnu/")
    endif()
  endif()

  if(NOT EXISTS "${TRT_SO_DIR}")
    message(FATAL_ERROR "${TRT_SO_DIR} not found")
  endif()
  target_link_libraries(i_seed_drone_onboard_core PUBLIC "${TRT_SO_DIR}/libnvinfer.so")
  target_link_libraries(i_seed_drone_onboard_core PUBLIC "${TRT_SO_DIR}/libnvonnxparser.so")
endif()

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
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
  )

  if(NOT CMAKE_CROSSCOMPILING)
    target_include_directories(
        i_seed_drone_onboard_core
        PUBLIC
        "/usr/include/libusb-1.0/"
    )
  endif()

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
endif()

if(I_SEED_DRONE_ONBOARD_PEDANTIC)
  # Disable warnings in automatically generated files
  file(
      COPY_FILE
      "${CMAKE_CURRENT_LIST_DIR}/clang-tidy-disabled"
      "${CMAKE_CURRENT_BINARY_DIR}/.clang-tidy"
  )

  if("${CLANG_TIDY_EXE}" STREQUAL "")
    set(CLANG_TIDY_EXE "clang-tidy")
  endif()

  if(I_SEED_DRONE_ONBOARD_MANIFOLD)
    set_target_properties(
        i_seed_drone_onboard
        PROPERTIES
        CXX_CLANG_TIDY "${CLANG_TIDY_EXE}"
    )
  endif()

  set_target_properties(
      i_seed_drone_onboard_core
      PROPERTIES
      CXX_CLANG_TIDY "${CLANG_TIDY_EXE}"
  )
endif()

if(I_SEED_DRONE_ONBOARD_MANIFOLD)
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

  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruslan Baratov <x@ruslo.dev>")
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "cuda-cudart-10-2, libboost-filesystem1.65.1, libboost-system1.65.1, libc6, libgcc1, libjpeg-turbo8, libnvinfer8, libnvonnxparsers8, libopencv, libprotobuf10, libstdc++6, libusb-1.0-0")
  include(CPack)
endif()

enable_testing()
add_test(
    NAME i_seed_drone_onboard_core_ut
    COMMAND i_seed_drone_onboard_core_ut
    "${CMAKE_CURRENT_LIST_DIR}"
)
