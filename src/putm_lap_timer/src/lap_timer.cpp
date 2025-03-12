#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class LapTimer : public rclcpp::Node
{
public:
    LapTimer() : Node("lap_timer"), last_lat(0.0), last_lon(0.0), last_lap_time(0), total_distance(0.0)
    {
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 50, std::bind(&LapTimer::gps_callback, this, std::placeholders::_1));
        lap_timer_pub_ = this->create_publisher<std_msgs::msg::Float32>("/putm_vcl/lap_timer/delta", 50);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&LapTimer::lap_timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_timer_pub_;

    double last_lat, last_lon;
    rclcpp::Time last_lap_time;
    rclcpp::Time best_lap_time_start, best_lap_time_end;
    double total_distance;
    double sector_lat, sector_lon;
    int sector_number = 0;
    int lap_count = 0;
    double closest = 10;
    double delta_time = 0;

    // Współrzędne start/meta
    const double EARTH_RADIUS = 6371000.0;
    const double START_LAT = 52.238843;
    const double START_LON = 16.230933;
    const double LAP_DISTANCE = 10.0; // Okrążenie co 75m
    const double DELTA_DISTANCE = 0.5;

    struct Sector
    {
        double lat, lon, time;
    };

    std::vector<Sector> reference_lap;
    std::vector<Sector> best_lap;
    bool is_first_lap = true;

    bool active = false;
    bool acc = false;

    double degreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    // Funkcja obliczająca odległość między dwoma punktami GPS
    // latitude1, longitude1 - współrzędne pierwszego punktu (stopnie)
    // latitude2, longitude2 - współrzędne drugiego punktu (stopnie)
    double haversineDistance(double latitude1, double longitude1, double latitude2, double longitude2)
    {
        // Konwersja stopni na radiany
        double lat1 = degreesToRadians(latitude1);
        double lon1 = degreesToRadians(longitude1);
        double lat2 = degreesToRadians(latitude2);
        double lon2 = degreesToRadians(longitude2);

        // Różnice współrzędnych
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        // Wzór Haversine'a
        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1) * cos(lat2) *
                       sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        // Obliczenie odległości
        return EARTH_RADIUS * c;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {

        double current_lat = msg->latitude;
        double current_lon = msg->longitude;
        rclcpp::Time now = this->now();

        // RCLCPP_INFO(this->get_logger(), " Odległość auta: %f, %d", haversineDistance(current_lat, current_lon, START_LAT, START_LON), active);
        // RCLCPP_INFO(this->get_logger(), " Odległość auta: %f, %f", current_lat, current_lon);

        // Tryb 1: Wykrywanie przejazdu przez linię start/meta
        double distance = haversineDistance(current_lat, current_lon, START_LAT, START_LON);

        if (distance < 1.4)
        {
            if (!active)
            {
                closest = distance;
                active = true;
            }
            else
            {
                if (distance < closest)
                {
                    closest = distance;
                }
                else if (!acc)
                {
                    if (last_lap_time.nanoseconds() != 0)
                    { // Pomijamy pierwszy pomiar
                        RCLCPP_INFO(this->get_logger(), "Okrążenie: %d Czas: %.3f s", lap_count, (now - last_lap_time).seconds());
                    }
                    lap_count++;
                    sector_number = 0; // Resetujemy licznik sektora
                    
                    switch (lap_count)
                    {
                    case 1:
                        best_lap_time_start = now;
                        break;
                    case 2:
                        best_lap_time_end = now;
                        best_lap = reference_lap;
                        break;
                    
                    default:
                        if ((best_lap_time_end - best_lap_time_start).seconds() - (now - last_lap_time).seconds() > 0){
                            best_lap_time_start = last_lap_time;
                            best_lap_time_end = now;
                            best_lap = reference_lap;
                        }   
                        break;
                    }
                    last_lap_time = now;
                    active = true;
                    acc = true;
                }
            }
        }
        else
        {
            closest = 10;
            active = false;
            acc = false;
        }

        if (lap_count == 1)
        {
            if (reference_lap.empty() || haversineDistance(current_lat, current_lon, reference_lap.back().lat, reference_lap.back().lon) >= DELTA_DISTANCE)
            {
                reference_lap.push_back({current_lat, current_lon, (now - last_lap_time).seconds()});
                // RCLCPP_INFO(this->get_logger(), "Zapisano punkt referencyjny #%lu", reference_lap.size());
            }
        }
        else
        {
            if (!best_lap.empty())
            {
                double min_distance = 10.0;
                int closest_index = -1;

                // Znajdź najbliższy punkt referencyjny
                for (size_t i = 0; i < best_lap.size(); i++)
                {
                    double d = haversineDistance(current_lat, current_lon, best_lap[i].lat, best_lap[i].lon);
                    if (d < min_distance)
                    {
                        min_distance = d;
                        closest_index = i;
                    }
                }

                // Jeśli znaleziono najbliższy punkt, oblicz deltę czasu
                if (closest_index != -1)
                {
                    double sector_time = (now - last_lap_time).seconds();
                    delta_time = sector_time - best_lap[closest_index].time;

                    RCLCPP_INFO(this->get_logger(), "Delta: %.3f s", delta_time);
                }
            }
        }

        last_lat = current_lat;
        last_lon = current_lon;
    }
    void lap_timer_callback()
    {
        auto message = std_msgs::msg::Float32();
        message.data = delta_time;
        lap_timer_pub_->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LapTimer>());
    rclcpp::shutdown();
    return 0;
}
