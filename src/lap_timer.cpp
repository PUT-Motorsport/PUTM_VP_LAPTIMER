#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
// #include "putm_vcl_interfaces/msg/lap_timer.hpp"
#include <cmath>
#include <chrono>
#include <sstream> // Do budowania nazwy pliku
#include <string>
#include <iomanip>
#include <fstream>
#include <ctime>
// Use the chrono_literals namespace for easier time manipulation
using namespace std::chrono_literals;

// Define the LapTimer class, which inherits from rclcpp::Node
class LapTimer : public rclcpp::Node
{
public:
    // Constructor for the LapTimer class
    LapTimer() : Node("lap_timer"), last_lat(0.0), last_lon(0.0), last_lap_time(0), total_distance(0.0)
    {
        // Create a subscription to the "/vectornav/gnss" topic with a QoS of 50
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 50, std::bind(&LapTimer::gps_callback, this, std::placeholders::_1));

        // Create publishers for the lap timer delta and time
        // lap_timer_pub = this->create_publisher<putm_vcl_interfaces::msg::LapTimer>("/putm_vcl/lap_timer", 50);

        // Create a timer that triggers every 20 milliseconds
        timer_ = this->create_wall_timer(
            20ms, std::bind(&LapTimer::lap_timer_callback, this));
        //creata a subscription to the "/vectronav/velocity_body" topic with a QoS of 50
        body_velocity_sub= this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/vectornav/velocity_body", 10,std::bind(&LapTimer::vel_bd_callback, this, std::placeholders::_1));
        //create csv file and trigger a header
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        //create file path with time included
        std::stringstream ss;
        ss << "/home/franiuu/PUTM_VP_LAPTIMER/log/laptimer_data_" 
        << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
        << ".csv";
        std::stringstream ss_constant;
        ss_constant << "/home/franiuu/PUTM_VP_LAPTIMER/log/laptimer_data_we_"
        << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
        <<".csv";

        std::string file_path_for_const=ss_constant.str();
        std::string file_path = ss.str();

        // open file
        file_.open(file_path);
        file_ << std::fixed << std::setprecision(10);
        file_const.open(file_path_for_const);
        file_const<< std::fixed << std::setprecision(10);
        //log if file is open or closed
        if(!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "Log file created: %s", file_path.c_str());
        }
    }

private:

    std::ofstream file_;
    std::ofstream file_const;
    std::string file_path_;
    // Timer object
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscription to the GPS topic
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    //Subscription to the Velocity body topic
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr body_velocity_sub;
    
    // Publishers for the lap timer delta and time
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr lap_timer_delta_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr lap_timer_time_pub_;
    // rclcpp::Publisher<putm_vcl_interfaces::msg::LapTimer>::SharedPtr lap_timer_pub;

    // Last known latitude and longitude
    double last_lat, last_lon;

    // Last lap time
    rclcpp::Time last_lap_time;

    // Best lap time start and end
    rclcpp::Time best_lap_time_start, best_lap_time_end;

    // Total distance traveled
    double total_distance;

    // Sector latitude and longitude
    double sector_lat, sector_lon;

    // Sector number
    int sector_number = 0;

    // Lap count
    int lap_count = 0;

    // Closest distance to the start/finish line
    double closest = 10;

    // Delta time
    double delta_time = 0;

    uint16_t lt = 0;
    uint16_t blt = 0;

    // Start/finish line coordinates
    const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
    const double START_LAT = 52.239048;    // Start latitude
    const double START_LON = 16.230333;    // Start longitude
    const double DELTA_DISTANCE = 0.5;     // Minimum distance between sectors

    //create every single point marker
    int current_checkpoint = 0;

    // Structure to represent a sector
    struct Sector
    {
        double lat, lon, time; // Latitude, longitude, and time
    };

    // Vectors to store the reference lap and best lap
    std::vector<Sector> reference_lap;
    std::vector<Sector> best_lap;
    std::vector<Sector> current_lap;

    // Flag to indicate if this is the first lap
    bool is_first_lap = true;

    // Flags to indicate if the lap timer is active and if the acceleration flag is set
    bool active = false;
    bool acc = false;

    // Function to convert degrees to radians
    double degreesToRadians(double degrees)
    {  
        // Convert degrees to radians using the formula: radians = degrees * pi / 180
        return degrees * M_PI / 180.0;
    }

    // Function to calculate the distance between two GPS points using the Haversine formula
    double haversineDistance(double latitude1, double longitude1, double latitude2, double longitude2)
    {
        // Convert latitudes and longitudes to radians
        double lat1 = degreesToRadians(latitude1);
        double lon1 = degreesToRadians(longitude1);
        double lat2 = degreesToRadians(latitude2);
        double lon2 = degreesToRadians(longitude2);

        // Calculate the differences between latitudes and longitudes
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        // Calculate the Haversine distance
        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1) * cos(lat2) *
                       sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        // Return the distance in meters
        return EARTH_RADIUS * c;
    }
    //spped callback function
    double current_spd=0.0;
    void vel_bd_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        current_spd = msg->twist.twist.linear.x;
    }


    // Callback function for the GPS subscription
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Get the current latitude and longitude
        double current_lat = msg->latitude;
        double current_lon = msg->longitude;
        
        // Get the current time
        rclcpp::Time now = this->now();

        // Calculate the distance to the start/finish line
        double distance = haversineDistance(current_lat, current_lon, START_LAT, START_LON);

        // Check if the vehicle is close to the start/finish line
        if (distance < 10)
        {
            // If this is the first time the vehicle is close to the start/finish line, set the active flag
            if (!active)
            {
                closest = distance;
                active = true;
            }
            else
            {
                // If the vehicle is getting closer to the start/finish line, update the closest distance
                if (distance < closest)
                {
                    closest = distance;
                }
                else if (!acc)
                {
                    // If the vehicle has completed a lap, print the lap time and update the lap count
                    if (last_lap_time.nanoseconds() != 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Lap: %d Time: %.3f s", lap_count, (now - last_lap_time).seconds());
                    }
                    lap_count++;
                    sector_number = 0;


                    // create current checkpoint and make new csv file which will make data for every single lap
                    auto t = std::time(nullptr);
                    auto tm = *std::localtime(&t);

                    std::stringstream ss;
                    ss << "/home/franiuu/PUTM_VP_LAPTIMER/log/laptimer_data_"
                    << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
                    << ".csv";
                    if (file_.is_open()){
                        file_.close();
                    }
                    file_path_ = ss.str();

                    file_.open(file_path_);
                    file_ << std::fixed << std::setprecision(10);

                    if (!file_.is_open()) {
                        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path_.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Log file created: %s", file_path_.c_str());
                        file_ << "checkpoint,delta,lat,lon,lap,speed\n";
                    }

  
                    // Update the best lap time if necessary
                    switch (lap_count)
                    {
                    case 1:
                        best_lap_time_start = now;
                        break;
                    case 2:
                        best_lap_time_end = now;
                        best_lap = current_lap;
                        reference_lap = current_lap;
                        break;
                    default:
                        reference_lap = current_lap;
                        if ((best_lap_time_end - best_lap_time_start).seconds() - (now - last_lap_time).seconds() > 0)
                        {
                            best_lap_time_start = last_lap_time;
                            best_lap_time_end = now;
                            best_lap = current_lap;
                        }
                        lt = static_cast<uint16_t>(1000 * (now - last_lap_time).seconds());
                        blt = static_cast<uint16_t>(1000 * (best_lap_time_end - best_lap_time_start).seconds());
                        break;
                    }
                    current_lap.clear();
                    last_lap_time = now;
                    active = true;
                    acc = true;
                }
            }
        }
        else
        {
            // If the vehicle is not close to the start/finish line, reset the closest distance and active flag
            closest = 10;
            active = false;
            acc = false;
        }

        // Update the reference lap if this is the first lap
        if (lap_count == 1)
        {
            if (current_lap.empty() || haversineDistance(current_lat, current_lon, current_lap.back().lat, current_lap.back().lon) >= DELTA_DISTANCE)
            {
                current_lap.push_back({current_lat, current_lon, (now - last_lap_time).seconds()});
            }
        }
        else if(lap_count > 1)
        {
            if (current_lap.empty() || haversineDistance(current_lat, current_lon, current_lap.back().lat, current_lap.back().lon) >= DELTA_DISTANCE)
            {
                current_lap.push_back({current_lat, current_lon, (now - last_lap_time).seconds()});
            }
            // If this is not the first lap, find the closest sector in the best lap
            if (!best_lap.empty())
            {
                double min_distance = 10.0;
                int closest_index = -1;

                for (size_t i = 0; i < best_lap.size(); i++)
                {
                    double d = haversineDistance(current_lat, current_lon, best_lap[i].lat, best_lap[i].lon);
                    if (d < min_distance)
                    {
                        min_distance = d;
                        closest_index = i;
                    }
                }

                // If a closest sector is found, calculate the delta time
                if (closest_index != -1)
                {
                    double sector_time = (now - last_lap_time).seconds();
                    delta_time = sector_time - best_lap[closest_index].time;

                    // RCLCPP_INFO(this->get_logger(), "Delta: %.3f s", delta_time);
                }
            }
        }

        // Update the last known latitude and longitude
        last_lat = current_lat;
        last_lon = current_lon;
        RCLCPP_INFO(this->get_logger(), "Delta:  %.3f, Current lap: %d, Best lap: %d, Lap count: %d\n", delta_time, lt, blt, lap_count);
        //draw data for laptimer
        if (file_.is_open())
        {
            current_checkpoint ++;
            file_const << current_checkpoint << "," << delta_time << "," << current_lat << "," << current_lon << "," << lap_count << "," << current_spd <<"\n";
            file_ << current_checkpoint << "," << delta_time << "," << current_lat << "," << current_lon << "," << lap_count << "," << current_spd <<"\n";
            file_const.flush();
            file_.flush();
        }
    }

    // Callback function for the lap timer
    void lap_timer_callback()
    {
        // Publish the delta time
        // auto message = putm_vcl_interfaces::msg::LapTimer();
        double del = 0.2322;
        // int16_t u = static_cast<int16_t>(del * 1000);
        int16_t u = -2137;
        // message.delta = (int16_t)delta_time;
        // message.current_lap = (uint16_t)lt;
        // message.best_lap = (uint16_t)blt;
        // message.lap_counter = (uint8_t)lap_count;
        // message.current_lap = 56145;
        // message.best_lap = 52123;
        // message.lap_counter = 11;
        // message.delta = u;
        // lap_timer_pub->publish(message);
        
    }
};

// Main function
int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a shared pointer to the LapTimer node
    auto node = std::make_shared<LapTimer>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
