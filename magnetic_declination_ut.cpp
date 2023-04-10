#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include "magnetic_declination.h"
#include "main_ut.h"  // gtest_project_top_directory

class magnetic_declination_test : public ::testing::Test {
 public:
  static std::string json_path() {
    boost::filesystem::path p{gtest_project_top_directory};
    BOOST_VERIFY(boost::filesystem::exists(p));
    p /= "data";
    p /= "step_5_igrf_grid_data.json";
    BOOST_VERIFY(boost::filesystem::exists(p));
    return p.string();
  }
};

TEST_F(magnetic_declination_test, simple) {
  magnetic_declination m{boost::gregorian::day_clock::local_day(), json_path(),
                         0.0, 0.0};
}

TEST_F(magnetic_declination_test, unreliable) {
  magnetic_declination m{boost::gregorian::day_clock::local_day(), json_path(),
                         90.0, 180.0};
}

TEST_F(magnetic_declination_test, cyprus) {
  const double lat{34.0 + 40.0 / 60.0 + 30.0 / 3600.0};
  const double lon{33.0 + 1.0 / 60.0 + 60.0 / 3600.0};

  boost::gregorian::date d_2022{2022, boost::gregorian::Dec, 31};
  magnetic_declination m_2022{d_2022, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2022.declination(), 5.4057);

  boost::gregorian::date d_2023{2023, boost::gregorian::Dec, 31};
  magnetic_declination m_2023{d_2023, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2023.declination(), 5.47491);

  boost::gregorian::date d_2024{2024, boost::gregorian::Dec, 31};
  magnetic_declination m_2024{d_2024, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2024.declination(), 5.54409);
}

TEST_F(magnetic_declination_test, bali) {
  const double lat{-1.0 * (8.0 + 39.0 / 60.0 + 0.0 / 3600.0)};
  const double lon{115.0 + 13.0 / 60.0 + 0.0 / 3600.0};

  boost::gregorian::date d_2022{2022, boost::gregorian::Dec, 31};
  magnetic_declination m_2022{d_2022, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2022.declination(), 0.87273);

  boost::gregorian::date d_2023{2023, boost::gregorian::Dec, 31};
  magnetic_declination m_2023{d_2023, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2023.declination(), 0.81848);

  boost::gregorian::date d_2024{2024, boost::gregorian::Dec, 31};
  magnetic_declination m_2024{d_2024, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2024.declination(), 0.76434);
}

TEST_F(magnetic_declination_test, hi) {
  const double lat{60.0};
  const double lon{-135.0};

  boost::gregorian::date d_2022{2022, boost::gregorian::Dec, 31};
  magnetic_declination m_2022{d_2022, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2022.declination(), 18.70824);
}

TEST_F(magnetic_declination_test, low) {
  const double lat{75.0};
  const double lon{-75.0};

  boost::gregorian::date d_2022{2022, boost::gregorian::Dec, 31};
  magnetic_declination m_2022{d_2022, json_path(), lat, lon};
  ASSERT_DOUBLE_EQ(m_2022.declination(), -36.8643);
}
