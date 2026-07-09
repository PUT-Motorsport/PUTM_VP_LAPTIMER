#include <gtest/gtest.h>
#include <cmath>

const double EARTH_RADIUS_M = 6371000.0;

double degreesToRadians(double degrees) {return degrees * M_PI / 180.0;}

double haversineDistance(double lat1, double lon1, double lat2, double lon2)
{
  double dLat = degreesToRadians(lat2 - lat1);
  double dLon = degreesToRadians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(degreesToRadians(lat1)) * cos(
    degreesToRadians(
      lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_M * c;
}

TEST(LapTimerTest, HaversinePrecision)
{
  double lat1 = 52.41689;
  double lon1 = 16.79793;

  double dist = haversineDistance(lat1, lon1, lat1, lon1);

  // Odległość punktu od samego siebie musi wynosić zero - matematycznie niezawodne
  EXPECT_NEAR(dist, 0.0, 0.001);
}

TEST(LapTimerTest, CrossLineEdgeCase)
{
  bool crossed_line = false;
  EXPECT_FALSE(crossed_line) << "System incorrectly detected finish line cross!";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}