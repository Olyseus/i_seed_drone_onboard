#include <gtest/gtest.h> // ::testing::Test

#include "home_altitude.h"

class home_altitude_test : public ::testing::Test {
};

TEST_F(home_altitude_test, double_start) {
  home_altitude h;

  h.mission_start();
  ASSERT_THROW(h.mission_start(), std::runtime_error);
}

TEST_F(home_altitude_test, double_stop) {
  home_altitude h;

  h.mission_start();
  h.mission_stop();

  ASSERT_THROW(h.mission_stop(), std::runtime_error);
}

TEST_F(home_altitude_test, no_start) {
  home_altitude h;

  ASSERT_THROW(h.set_altitude(0.0, 0.0, 0.0), std::runtime_error);
  ASSERT_THROW(h.get_home_altitude(), std::runtime_error);
  ASSERT_THROW(h.mission_stop(), std::runtime_error);
}

TEST_F(home_altitude_test, bad_mission_altitude) {
  home_altitude h;

  h.mission_start();
  h.set_altitude(0.0, 0.0, 0.0);
  ASSERT_THROW(h.set_altitude(0.0, 1.0, 0.0), std::runtime_error);
}

TEST_F(home_altitude_test, bad_home_altitude) {
  home_altitude h;

  h.mission_start();
  h.set_altitude(0.0, 0.0, 0.0);
  ASSERT_THROW(h.set_altitude(0.0, 0.0, 1.0), std::runtime_error);
}

TEST_F(home_altitude_test, bad_drone_altitude) {
  home_altitude h;

  constexpr double mission_altitude{10.0};
  constexpr double home_altitude{10.0};

  h.mission_start();
  h.set_altitude(20.1, mission_altitude, home_altitude);
  h.set_altitude(20.2, mission_altitude, home_altitude);

  ASSERT_THROW(h.set_altitude(21.5, mission_altitude, home_altitude), std::runtime_error);
}

TEST_F(home_altitude_test, no_data) {
  home_altitude h;

  h.mission_start();

  ASSERT_THROW(h.get_home_altitude(), std::runtime_error);
}

TEST_F(home_altitude_test, simple) {
  home_altitude h;

  {
    constexpr double mission_altitude{10.0};
    constexpr double home_altitude{90.0};

    h.mission_start();

    h.set_altitude(100.0, mission_altitude, home_altitude);
    h.set_altitude(100.1, mission_altitude, home_altitude);
    h.set_altitude(100.2, mission_altitude, home_altitude);

    ASSERT_DOUBLE_EQ(h.get_home_altitude(), home_altitude);
  }

  h.mission_stop();

  {
    constexpr double mission_altitude{20.0};
    constexpr double home_altitude{5.0};

    h.mission_start();

    h.set_altitude(25.0, mission_altitude, home_altitude);
    h.set_altitude(25.3, mission_altitude, home_altitude);
    h.set_altitude(25.1, mission_altitude, home_altitude);
    h.set_altitude(25.2, mission_altitude, home_altitude);

    ASSERT_DOUBLE_EQ(h.get_home_altitude(), home_altitude);
  }
}
