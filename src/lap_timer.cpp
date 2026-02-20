#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <cmath>
#include <chrono>
#include <memory>
#include <sstream>
#include <array>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <cstdlib>

using namespace std::chrono_literals;

class LapTimer : public rclcpp::Node
{
private:
    // Define the states for the lap timer
    enum class State
    {
        WAITING_FOR_START,
        RECORDING_REFERENCE_LAP,
        LAPPING
    };

public:
    LapTimer() : Node("lap_timer"), m_state(State::WAITING_FOR_START)
    {
        // ROS2 Subscribers and Publishers
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 50, std::bind(&LapTimer::gps_callback, this, std::placeholders::_1));
        //creata a subscription to the "/vectronav/velocity_body" topic with a QoS of 50
        body_velocity_sub= this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
             "/vectornav/velocity_body", 10,std::bind(&LapTimer::vel_bd_callback, this, std::placeholders::_1));

        lap_timer_pub_ = this->create_publisher<std_msgs::msg::String>("/putm_vcl/lap_timer", 50);

        // ROS2 Wall timer for periodic publishing
        m_wall_timer = this->create_wall_timer(
            20ms, std::bind(&LapTimer::lap_timer_callback, this));

        // Initial gate post calculation for the start line
        setup_gate_posts(START_CENTER_LAT, START_CENTER_LON, START_HEADING_DEG, 10.0);

        const char* home_dir = std::getenv("HOME");
        std::string base_path_str;

        if (home_dir) 
        {
            base_path_str = std::string(home_dir) + "/PUTM_VP_LAPTIMER/csv_logs/";
        } 
        else 
        {
            base_path_str = "/tmp/PUTM_VP_LAPTIMER/csv_logs/";
        }

        std::filesystem::path log_dir(base_path_str);

        RCLCPP_INFO(this->get_logger(), "Log directory path: %s", log_dir.string().c_str());
        try {
                if (!std::filesystem::exists(log_dir)) {
                    std::filesystem::create_directories(log_dir);
                    RCLCPP_INFO(this->get_logger(), "Created log directory: %s", log_dir.string().c_str());
                }
                else { 
                    RCLCPP_INFO(this->get_logger(), "Directory already exists: %s", log_dir.string().c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error creating log directory: %s", e.what());
            }
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        
        //stworzenie drugiej nazwy pliku z aktualną datą i godziną dla całej sesji
        std::stringstream ss_filename_session;
        ss_filename_session << "SESSION_TOTAL_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
        std::filesystem::path full_path_const = log_dir / ss_filename_session.str();

        log_dir_ = log_dir;
        session_file_.open(full_path_const);

        if (session_file_.is_open()) {
            session_file_ << std::fixed << std::setprecision(10);
            session_file_ << "timestamp,lap_count,lat,lon,speed,state\n";
            RCLCPP_INFO(this->get_logger(), "Session log file created: %s", full_path_const.string().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not open session file: %s", full_path_const.string().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "LapTimer initialized, waiting for first start line cross.");
    }

    ~LapTimer() 
    {
        if (lap_file_.is_open()) lap_file_.close();
        if (session_file_.is_open()) session_file_.close();
        RCLCPP_INFO(this->get_logger(), "Log files closed safely.");
    }

private:
    std::filesystem::path log_dir_;
    std::ofstream lap_file_;
    std::ofstream session_file_;  

    // -- ROS2 Constructs --
    rclcpp::TimerBase::SharedPtr m_wall_timer;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lap_timer_pub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr body_velocity_sub;

    double current_spd=0.0;

    // -- State Machine --
    State m_state;
    // bool m_is_approaching_start = false;
    // double m_closest_approach = 10.0;
    // bool m_has_crossed_this_pass = false;

    // -- Lap Data --
    uint8_t m_lap_count = 0;
    uint32_t m_last_lap_time_ms = 0;
    uint32_t m_best_lap_time_ms = 0;
    double m_delta_time_s = 0.0;
    rclcpp::Time m_current_lap_start_time;

    struct Sector
    {
        double lat, lon, time_s;
    };

    static constexpr size_t MAX_SECTORS = 10000;

    std::array<Sector, MAX_SECTORS> m_reference_lap_sectors;
    size_t m_ref_sectors_count = 0; // Licznik, ile mamy aktualnie punktów

    std::array<Sector, MAX_SECTORS> m_best_lap_sectors;
    size_t m_best_sectors_count = 0; // Licznik dla najlepszego okrążenia

    // -- Constants --
    const double EARTH_RADIUS_M = 6371000.0;
    const double START_CENTER_LAT = 52.239048;    
    const double START_CENTER_LON = 16.230333;   
    const double SECTOR_RECORDING_DISTANCE_M = 0.5; // Minimum distance to record a new sector point

    const double START_HEADING_DEG = 292.0; // This is the heading calculated from rosbag data for the start line (direction NW approximately) 
    //This need to be calculated based on the actual start line orientation. It can be calculated from the two gate posts or from the rosbag data as done here.

    //Deratives for gate coridnations
    double m_gate_p1_lat;
    double m_gate_p1_lon;
    double m_gate_p2_lat;
    double m_gate_p2_lon;

    // Deratives for gate post calculation
    double m_prev_lat = 0.0;
    double m_prev_lon = 0.0;

    // -- Utility Functions --
    //Function to calculate gate post coordinates based on center point, heading and half width of the gate
    void setup_gate_posts(double center_lat, double center_lon, double heading_deg, double half_width_m)
    {
        double lat_rad = degreesToRadians(center_lat);
        double hdg_rad = degreesToRadians(heading_deg);

        // P1 (Left cone) = Heading - 90 degrees
        double left_angle = hdg_rad - (M_PI / 2.0);
        m_gate_p1_lat = center_lat + (half_width_m * cos(left_angle) / EARTH_RADIUS_M) * (180.0 / M_PI);
        m_gate_p1_lon = center_lon + (half_width_m * sin(left_angle) / (EARTH_RADIUS_M * cos(lat_rad))) * (180.0 / M_PI);

        // P2 (Right cone) = Heading + 90 degrees
        double right_angle = hdg_rad + (M_PI / 2.0);
        m_gate_p2_lat = center_lat + (half_width_m * cos(right_angle) / EARTH_RADIUS_M) * (180.0 / M_PI);
        m_gate_p2_lon = center_lon + (half_width_m * sin(right_angle) / (EARTH_RADIUS_M * cos(lat_rad))) * (180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "Gate Calculated: P1(%.8f, %.8f) - P2(%.8f, %.8f)", 
            m_gate_p1_lat, m_gate_p1_lon, m_gate_p2_lat, m_gate_p2_lon);
    }


    double degreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    double haversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        double dLat = degreesToRadians(lat2 - lat1);
        double dLon = degreesToRadians(lon2 - lon1);
        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(degreesToRadians(lat1)) * cos(degreesToRadians(lat2)) *
                       sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return EARTH_RADIUS_M * c;
    }

    // Funkcja pomocnicza: Iloczyn wektorowy (Cross Product)
    double crossProduct(double ax, double ay, double bx, double by, double cx, double cy)
    {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }

    // Sprawdza matematycznie czy przecięliśmy linię między P1 a P2
    bool check_line_crossing(double curr_lat, double curr_lon)
    {
        if (m_prev_lat == 0.0 && m_prev_lon == 0.0) return false;

        // Używamy dynamicznie wyliczonych punktów
        double p1_x = m_gate_p1_lon;
        double p1_y = m_gate_p1_lat;
        double p2_x = m_gate_p2_lon;
        double p2_y = m_gate_p2_lat;

        double car_prev_x = m_prev_lon;
        double car_prev_y = m_prev_lat;
        double car_curr_x = curr_lon;
        double car_curr_y = curr_lat;

        double cp1 = crossProduct(p1_x, p1_y, p2_x, p2_y, car_prev_x, car_prev_y);
        double cp2 = crossProduct(p1_x, p1_y, p2_x, p2_y, car_curr_x, car_curr_y);
        double cp3 = crossProduct(car_prev_x, car_prev_y, car_curr_x, car_curr_y, p1_x, p1_y);
        double cp4 = crossProduct(car_prev_x, car_prev_y, car_curr_x, car_curr_y, p2_x, p2_y);

        if (((cp1 > 0 && cp2 < 0) || (cp1 < 0 && cp2 > 0)) &&
            ((cp3 > 0 && cp4 < 0) || (cp3 < 0 && cp4 > 0)))
        {
            return true;
        }

        return false;
    }

    // -- State Handlers --

    void process_lap_crossing(const rclcpp::Time &now)
    {
        // The very first time we cross the line, we just start the timer and state.
        if (m_state == State::WAITING_FOR_START)
        {
            m_lap_count = 1;
            m_current_lap_start_time = now;
            m_state = State::RECORDING_REFERENCE_LAP;
            RCLCPP_INFO(this->get_logger(), "Crossed start line for the first time. Starting lap 1.");
        }
        else 
        {
            // If we are here, a lap has been completed.
            double completed_lap_time_s = (now - m_current_lap_start_time).seconds();
            m_last_lap_time_ms = static_cast<uint32_t>(completed_lap_time_s * 1000);
            
            RCLCPP_INFO(this->get_logger(), "Lap %d finished. Time: %.3f s", m_lap_count, completed_lap_time_s);

            bool is_new_best = false;

            if (m_state == State::RECORDING_REFERENCE_LAP)
            {
                is_new_best = true; // First lap is always the best lap
                m_state = State::LAPPING; // Transition to lapping state after recording the reference lap
            }
            else if (m_state == State::LAPPING && m_last_lap_time_ms < m_best_lap_time_ms)
            {
                is_new_best = true;
            }

            if (is_new_best)
            {
                m_best_lap_time_ms = m_last_lap_time_ms;
                m_best_sectors_count = m_ref_sectors_count; // Update best sectors count
                std::copy(m_reference_lap_sectors.begin(), m_reference_lap_sectors.begin() + m_ref_sectors_count, m_best_lap_sectors.begin());
                RCLCPP_INFO(this->get_logger(), "New best lap recorded with time: %.3f s", completed_lap_time_s);
            }

            // Prepare for the next lap
            m_lap_count++;
            m_current_lap_start_time = now;
            m_ref_sectors_count = 0;
        }
        if (lap_file_.is_open()) {
            lap_file_.close();
        }

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream ss;
        ss << "LAP_" << std::setw(2) << std::setfill('0') << static_cast<int>(m_lap_count) 
        << "_" << std::put_time(&tm, "%H-%M-%S") << ".csv";

        std::filesystem::path new_lap_path = log_dir_ / ss.str();
        lap_file_.open(new_lap_path.string());

        if (lap_file_.is_open()) {
            lap_file_ << std::fixed << std::setprecision(10);
            lap_file_ << "time_into_lap,lat,lon,speed,delta\n"; // Nagłówek dla nowego pliku
            RCLCPP_INFO(this->get_logger(), "Started logging new lap file: %s", ss.str().c_str());
        }
    }

    void state_recording_reference_lap(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        double time_into_lap_s = (now - m_current_lap_start_time).seconds();

        bool should_record = false;

        if (m_ref_sectors_count == 0) 
        {
            should_record = true;
        } 
        else 
        {
            const auto& last_sector = m_reference_lap_sectors[m_ref_sectors_count - 1];
            
            if (haversineDistance(current_lat, current_lon, last_sector.lat, last_sector.lon) >= SECTOR_RECORDING_DISTANCE_M)
            {
                should_record = true;
            }
        }

        if (should_record)
        {
            if (m_ref_sectors_count < MAX_SECTORS)
            {
                m_reference_lap_sectors[m_ref_sectors_count] = {current_lat, current_lon, time_into_lap_s};
                m_ref_sectors_count++; 
            }
            else
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "MEMORY FULL! Increase MAX_SECTORS.");
            }
        }   
        

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Lap: %d | Time: %.2f s | (Recording Reference)", 
            m_lap_count, time_into_lap_s);
    }

    void state_lapping(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        double time_into_lap_s = (now - m_current_lap_start_time).seconds();

        bool should_record = false;

        if (m_ref_sectors_count == 0) 
        {
            should_record = true;
        } 
        else 
        {
            const auto& last_sector = m_reference_lap_sectors[m_ref_sectors_count - 1];
            
            if (haversineDistance(current_lat, current_lon, last_sector.lat, last_sector.lon) >= SECTOR_RECORDING_DISTANCE_M)
            {
                should_record = true;
            }
        }

        if (should_record)
        {
            if (m_ref_sectors_count < MAX_SECTORS)
            {
                m_reference_lap_sectors[m_ref_sectors_count] = {current_lat, current_lon, time_into_lap_s};
                m_ref_sectors_count++; 
            }
            else
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "MEMORY FULL! Increase MAX_SECTORS.");
            }
        } 

        if (m_best_sectors_count == 0)
        {
            m_delta_time_s = 0.0;
            return;
        }

        double min_dist_to_sector = 100.0;
        int closest_sector_idx = -1;

        for (size_t i = 0; i < m_best_sectors_count; ++i)
        {
            double d = haversineDistance(current_lat, current_lon, m_best_lap_sectors[i].lat, m_best_lap_sectors[i].lon);
            if (d < min_dist_to_sector)
            {
                min_dist_to_sector = d;
                closest_sector_idx = i;
            }
        }

        if (closest_sector_idx != -1 && min_dist_to_sector < 20.0)
        {
            double current_time_into_lap_s = (now - m_current_lap_start_time).seconds();
            double best_lap_time_at_sector_s = m_best_lap_sectors[closest_sector_idx].time_s;
            
            m_delta_time_s = current_time_into_lap_s - best_lap_time_at_sector_s;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Lap: %d | Time: %.2f s | Delta: %.3f s", 
                m_lap_count, current_time_into_lap_s, m_delta_time_s);
        }
    }
//Main callback for GPS data, handling state transitions and lap timing logic

    void vel_bd_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        current_spd = msg->twist.twist.linear.x;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        double current_lat = msg->latitude;
        double current_lon = msg->longitude;
        rclcpp::Time now = this->get_clock()->now();

        //handle_start_finish_crossing(current_lat, current_lon, now);
        if (check_line_crossing(current_lat, current_lon))
        {
            process_lap_crossing(now);
        }    
        // State-specific logic
        switch (m_state)
        {
        case State::WAITING_FOR_START:
            break;
        case State::RECORDING_REFERENCE_LAP:
            state_recording_reference_lap(current_lat, current_lon, now);
            break;
        case State::LAPPING:
            state_lapping(current_lat, current_lon, now);
            break;
        }

        //RCLCPP_INFO(this->get_logger(), "Delta: %.3f, Last lap: %d, Best lap: %d, Lap count: %d",
                    //m_delta_time_s, m_last_lap_time_ms, m_best_lap_time_ms, m_lap_count);
        m_prev_lat = current_lat;
        m_prev_lon = current_lon;

        if (session_file_.is_open()) 
        {
        session_file_ << now.seconds() << "," 
                      << static_cast<int>(m_lap_count) << "," 
                      << current_lat << "," << current_lon << "," 
                      << current_spd << "," << static_cast<int>(m_state) << "\n";
        session_file_.flush();
        }

        if (lap_file_.is_open() && m_state != State::WAITING_FOR_START) 
        {
            lap_file_ << (now - m_current_lap_start_time).seconds() << "," 
                    << current_lat << "," << current_lon << "," 
                    << current_spd << "," << m_delta_time_s << "\n";
            lap_file_.flush();
    }
    }

    void lap_timer_callback()
    {
        auto message = std_msgs::msg::String();
        std::stringstream ss;
        ss << "best_lap:" << m_best_lap_time_ms
           << ",lap_counter:" << static_cast<int>(m_lap_count)
           << ",last_lap:" << m_last_lap_time_ms
           << ",delta:" << static_cast<int32_t>(m_delta_time_s * 1000);
        message.data = ss.str();
        lap_timer_pub_->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Add use_sim_time parameter
    auto node = std::make_shared<LapTimer>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}