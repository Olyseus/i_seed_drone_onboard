#include "main_ut.h"  // gtest_project_top_directory

#include <gtest/gtest.h> // testing::InitGoogleTest

#include <iostream>  // std::cerr

const char* gtest_project_top_directory{nullptr};

auto main(int argc, char** argv) -> int {
  testing::InitGoogleTest(&argc, argv);
  if (argc != 2) {
    std::cerr << "argc: " << argc << std::endl;
    return EXIT_FAILURE;
  }

  gtest_project_top_directory = argv[1];
  std::cout << "Project top directory: " << gtest_project_top_directory << std::endl;

  return RUN_ALL_TESTS();
}
