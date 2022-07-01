cmake_minimum_required(VERSION 3.12)
project(i_seed_drone_onboard)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "C++ standard required")
set(CMAKE_CXX_EXTENSIONS FALSE CACHE BOOL "C++ standard extension")

option(I_SEED_DRONE_ONBOARD_PEDANTIC "Enable pedantic checks" OFF)

# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)

add_library(
    i_seed_drone_onboard_core
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
