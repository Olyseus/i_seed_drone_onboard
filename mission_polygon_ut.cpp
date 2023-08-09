#include <gtest/gtest.h>  // TEST_F

#include "mission_polygon.h"
#include "mission_simple_polygon.h"

class mission_polygon_test : public ::testing::Test {
 public:
  std::vector<mission_simple_polygon>& simple_polygons(mission_polygon& m) {
    return m.simple_polygons_;
  }
};

TEST_F(mission_polygon_test, triangle) {
  utils::polygon poly;
  poly.push_back({-88.99640326012238, 533.260107945901});
  poly.push_back({480.41295593388867, -159.04952881636052});
  poly.push_back({-391.416552670383, -374.21057912836795});

  mission_polygon mission(poly);
  ASSERT_EQ(simple_polygons(mission).size(), 1);
}

TEST_F(mission_polygon_test, 4_points_2_polygons) {
  utils::polygon poly;
  poly.push_back({1, 1});
  poly.push_back({1, -1});
  poly.push_back({-1, 1});
  poly.push_back({-1, -1});

  mission_polygon mission(poly);
  ASSERT_EQ(simple_polygons(mission).size(), 2);
}

TEST_F(mission_polygon_test, coordinates_for_simulator) {
  utils::polygon poly;

  poly.push_back({-6.598646985268483, -10.00870200892634});
  poly.push_back({6.598646985656046, -10.008702008515277});
  poly.push_back({6.598623291218747, 10.008702008039176});
  poly.push_back({-6.598623290763276, 10.008702007695618});

  mission_polygon mission(poly);
  auto path{mission.make({6.829418543029466e-11, -1.2293522473970883e-06})};
  ASSERT_EQ(path.size(), 3);
}

TEST_F(mission_polygon_test, real_1) {
  utils::polygon poly;
  poly.push_back({-46.59007849081423, -13.758538302038106});
  poly.push_back({-43.763067483994824, 42.64199859802392});
  poly.push_back({7.886133782995238, 46.7098205927571});
  poly.push_back({54.49661050302156, 6.183604275714031});
  poly.push_back({55.4493379506785, -33.731217439739915});
  poly.push_back({-27.478936270297655, -48.04566771941828});

  mission_polygon mission(poly);
  auto path{mission.make({-0.42463097200274963, 2.2625433533051056})};
  ASSERT_GT(path.size(), 0);
}

TEST_F(mission_polygon_test, real_2) {
  utils::polygon poly;
  poly.push_back({-42.94650548538668, -36.74136482468726});
  poly.push_back({3.7562000439808547, -3.8309061151506087});
  poly.push_back({49.84424514263499, -3.830772264681507});
  poly.push_back({-10.65393970252852, 44.40304320502546});

  mission_polygon mission(poly);
}

TEST_F(mission_polygon_test, real_3) {
  utils::polygon poly;

  poly.push_back({-112.7361158770866, -20.343643611643934});
  poly.push_back({-54.48050238261319, 25.167678410088783});
  poly.push_back({5.187949846362892, -34.65858759989709});
  poly.push_back({59.47978702576976, -32.15034223082682});
  poly.push_back({82.8611655363018, 43.06059224173675});
  poly.push_back({53.026645697455756, 107.16796164496597});
  poly.push_back({-12.048862324867837, 87.8377127288011});
  poly.push_back({11.302218625621828, 41.9897245321108});
  poly.push_back({59.14203230528836, -67.26317774085125});
  poly.push_back({-49.10425992618719, -101.64213200763196});
  poly.push_back({-24.27759775436021, 22.292479568449615});
  poly.push_back({2.3304769126344858, -7.070019440027647});
  poly.push_back({-20.682937680515046, -64.38824647954748});

  mission_polygon mission(poly);
}

TEST_F(mission_polygon_test, real_4) {
  utils::polygon poly;

  poly.push_back({-6.9311905901881445, 66.44691621380574});
  poly.push_back({13.93101545622654, 146.85634025744096});
  poly.push_back({30.49197419747042, 60.176896652693614});
  poly.push_back({90.34422724925594, 84.49289194965534});
  poly.push_back({50.55569098644225, 18.610919432152976});
  poly.push_back({100.57661136962395, -2.9822998292180287});
  poly.push_back({52.15370043377894, -33.96628099157287});
  poly.push_back({87.97999775980576, -82.99551816265594});
  poly.push_back({28.12676568311443, -144.56606813868837});
  poly.push_back({-12.830598488226077, -72.41311816210403});
  poly.push_back({-64.019837151203, -143.00598383941758});
  poly.push_back({-45.122985194805906, -40.63407144570294});
  poly.push_back({-101.04366548071845, -76.32760063850004});
  poly.push_back({-42.756937744779705, 0.16759953571457162});
  poly.push_back({-101.84116175191373, 47.63726091265737});
  poly.push_back({-18.145917457416342, 28.979393157396032});
  poly.push_back({-61.467689254323645, 143.52272309473219});

  mission_polygon mission(poly);
}

TEST_F(mission_polygon_test, real_5) {
  utils::polygon poly;

  poly.push_back({-26.546753551100927, -20.931004182778413});
  poly.push_back({39.3284608963488, 1.2132979075017927});
  poly.push_back({-12.781707345522099, 19.717706273167096});

  mission_polygon mission(poly);
  auto path{mission.make({0.8761104842678353, -19.713491401812213})};
  ASSERT_GT(path.size(), 0);
}

TEST_F(mission_polygon_test, real_6) {
  utils::polygon poly;

  poly.push_back({-10.997725230316865, 4.821035436104025e-06});
  poly.push_back({-3.665908410098509, -4.821771535738712e-06});
  poly.push_back({3.6659084101330532, -4.821698908935239e-06});
  poly.push_back({10.99772523056873, 4.820921451536602e-06});

  mission_polygon mission(poly);
  auto path{mission.make({0.8761104842678353, -19.713491401812213})};
  ASSERT_EQ(path.size(), 0);
}
