#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>        
#include <sensor_msgs/point_cloud2_iterator.hpp>  
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
 
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <string>
 
using namespace std::chrono_literals;

// Vehicle drive states
const std::string CLEAR      = "CLEAR";
const std::string CAUTION    = "CAUTION";
const std::string BRAKING    = "BRAKING";
const std::string REVERSING  = "REVERSING";
const std::string RECOVERING = "RECOVERING";
 
struct ObstaclePoint {
    float x, y, dist, angle;
};
 
struct Region {
    float a_min, a_max; // Angular bounds in degrees
};
 
class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance") {

		//  geometry and motion params
        this->declare_parameter("vehicle_width",    1.4);
        this->declare_parameter("vehicle_length",   2.0);
        this->declare_parameter("max_deceleration", 3.0);
        this->declare_parameter("reaction_time",    0.5);
        this->declare_parameter("wheelbase",        1.3);

		// reverse params
        this->declare_parameter("reverse_speed",      0.1);
        this->declare_parameter("reverse_duration",   1.5);
        this->declare_parameter("reverse_clear_dist", 1.0);
 
        vehicle_width_      = this->get_parameter("vehicle_width").as_double();
        vehicle_length_     = this->get_parameter("vehicle_length").as_double();
        max_decel_          = this->get_parameter("max_deceleration").as_double();
        reaction_time_      = this->get_parameter("reaction_time").as_double();
        wheelbase_          = this->get_parameter("wheelbase").as_double();
        reverse_speed_      = this->get_parameter("reverse_speed").as_double();
        reverse_duration_   = this->get_parameter("reverse_duration").as_double();
        reverse_clear_dist_ = this->get_parameter("reverse_clear_dist").as_double();

		// Speed thresholds
        min_safe_distance_  = 0.55;
        comfort_factor_     = 1.5;
        normal_speed_       = 0.3;
        slow_speed_         = 0.15;
        crawl_speed_        = 0.08;

		// Steering limits
        max_steering_angle_ = 0.52;
        normal_steering_    = 0.35;
        gentle_steering_    = 0.17;
 
        current_velocity_      = 0.0;
        current_steering_      = 0.0;
        emergency_stop_active_ = false;
        drive_state_           = CLEAR;
        reverse_start_time_    = this->now();
        last_reverse_time_     = this->now();
        recover_steer_dir_     = 0.0f;

		// angular regions around the vehicle
        regions_["front_center"] = {-10.0f,  10.0f};
        regions_["front_left"]   = { 10.0f,  30.0f};
        regions_["front_right"]  = {-30.0f, -10.0f};
        regions_["left"]         = { 30.0f,  90.0f};
        regions_["right"]        = {-90.0f, -30.0f};
        regions_["rear"]         = {150.0f, -150.0f};
 
        for (auto& kv : regions_)
            region_min_[kv.first] = 999.9f;

		// Local costmap centered on robot
        map_width_    = 100;
        map_height_   = 100;
        resolution_   = 0.1f;
        robot_cell_x_ = map_width_  / 2;
        robot_cell_y_ = map_height_ / 2;
        costmap_data_.assign(map_width_ * map_height_, 0);
 

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/velodyne_points", rclcpp::SensorDataQoS(),
            std::bind(&ObstacleAvoidance::scanCallback, this, std::placeholders::_1));
 

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_surround", rclcpp::SensorDataQoS(),
            std::bind(&ObstacleAvoidance::cloudCallback, this, std::placeholders::_1));
 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,  
            std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));
 
        cmd_pub_       = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd_vel", 10);
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        costmap_pub_   = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
        twist_pub_     = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_twist", 10);
 
        costmap_timer_ = this->create_wall_timer(500ms, std::bind(&ObstacleAvoidance::publishCostmap, this));
        safety_timer_  = this->create_wall_timer(100ms, std::bind(&ObstacleAvoidance::safetyMonitor, this));
 
        RCLCPP_INFO(this->get_logger(), "OBSTACLE AVOIDANCE STARTED");
	RCLCPP_INFO(this->get_logger(), "Vehicle: %.2fm x %.2fm | Max decel: %.2f m/s^2",
                    vehicle_width_, vehicle_length_, max_decel_);
        RCLCPP_INFO(this->get_logger(), "Reverse: %.2fm/s for %.2fs | Rear clear: %.2fm",
                    reverse_speed_, reverse_duration_, reverse_clear_dist_);
        RCLCPP_INFO(this->get_logger(), "Publishing: /cmd_vel");
    }
 
    void shutdownStop() {
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = this->now();
        cmd.drive.speed  = 0.0f;
        cmd_pub_->publish(cmd);
    }
 
 
private:
 
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_velocity_ = std::sqrt(vx * vx + vy * vy);
    }
 

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        for (auto& kv : region_min_)
            kv.second = 999.9f;
        std::fill(costmap_data_.begin(), costmap_data_.end(), 0);
 
        std::vector<ObstaclePoint> frame_obstacles;
 
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
 
            // Skip invalid readings
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max)
                continue;
 
            // angle in radians then convert to degrees
            float angle_rad = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
 
            // 
            float x = range * std::cos(angle_rad);
            float y = range * std::sin(angle_rad);
 
            frame_obstacles.push_back({x, y, range, angle_deg});
 
    
            for (auto& kv : regions_) {
                const std::string& name = kv.first;
                float a_min = kv.second.a_min;
                float a_max = kv.second.a_max;
                if ((a_min <= angle_deg && angle_deg <= a_max) ||
                    (name == "rear" && (angle_deg > 150.0f || angle_deg < -150.0f))) {
                    if (range < region_min_[name])
                        region_min_[name] = range;
                }
            }
 
            // Update costmap and inflate
            int cell_x = static_cast<int>(robot_cell_x_ + (x / resolution_));
            int cell_y = static_cast<int>(robot_cell_y_ + (y / resolution_));
            if (cell_x >= 0 && cell_x < map_width_ && cell_y >= 0 && cell_y < map_height_) {
                costmap_data_[cell_y * map_width_ + cell_x] = 100;
                inflateObstacleDynamic(cell_x, cell_y);
            }
        }
 
        obstacle_history_.push_back(frame_obstacles);
        if (static_cast<int>(obstacle_history_.size()) > 5)
            obstacle_history_.pop_front();
 
        updateConfirmedObstacles();
 
        bool corridor_safe = checkSafetyCorridor();
        makeVDecision(corridor_safe);
    }
 

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        for (auto& kv : region_min_)
            kv.second = 999.9f;
        std::fill(costmap_data_.begin(), costmap_data_.end(), 0);
 
        std::vector<ObstaclePoint> frame_obstacles;
 
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
 
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
 
            float ground_threshold  = -0.3f + (0.1f * std::fabs(x));
            float ceiling_threshold = 2.0f;
            if (z < ground_threshold || z > ceiling_threshold) continue;
 
            float dist = std::sqrt(x * x + y * y);
            if (dist > 50.0f) continue;
 
            float angle = static_cast<float>(std::atan2(y, x) * 180.0 / M_PI);
            frame_obstacles.push_back({x, y, dist, angle});
 
            for (auto& kv : regions_) {
                const std::string& name = kv.first;
                float a_min = kv.second.a_min;
                float a_max = kv.second.a_max;
                if ((a_min <= angle && angle <= a_max) ||
                    (name == "rear" && (angle > 150.0f || angle < -150.0f))) {
                    if (dist < region_min_[name])
                        region_min_[name] = dist;
                }
            }
 
            int cell_x = static_cast<int>(robot_cell_x_ + (x / resolution_));
            int cell_y = static_cast<int>(robot_cell_y_ + (y / resolution_));
            if (cell_x >= 0 && cell_x < map_width_ && cell_y >= 0 && cell_y < map_height_) {
                costmap_data_[cell_y * map_width_ + cell_x] = 100;
                inflateObstacleDynamic(cell_x, cell_y);
            }
        }
 
        obstacle_history_.push_back(frame_obstacles);
        if (static_cast<int>(obstacle_history_.size()) > 5)
            obstacle_history_.pop_front();
 
        updateConfirmedObstacles();
 
        bool corridor_safe = checkSafetyCorridor();
        makeVDecision(corridor_safe);
    }

//obstacle is confirmed only if seen in 2 of the last frames
    void updateConfirmedObstacles() {
        if (static_cast<int>(obstacle_history_.size()) < 3) return;
        confirmed_obstacles_.clear();
        for (auto& kv : regions_) {
            const std::string& region = kv.first;
            std::vector<float> distances;
            for (auto& frame : obstacle_history_) {
                float min_dist = 999.9f;
                bool found = false;
                for (auto& obs : frame) {
                    if (pointInRegion(obs.angle, region)) {
                        if (obs.dist < min_dist) {
                            min_dist = obs.dist;
                            found    = true;
                        }
                    }
                }
                if (found) distances.push_back(min_dist);
            }
            if (static_cast<int>(distances.size()) >= 2)
                confirmed_obstacles_[region] = *std::min_element(distances.begin(), distances.end());
        }
    }
 
    bool pointInRegion(float angle, const std::string& region_name) {
        float a_min = regions_[region_name].a_min;
        float a_max = regions_[region_name].a_max;
        if (region_name == "rear")
            return angle > 150.0f || angle < -150.0f;
        return a_min <= angle && angle <= a_max;
    }

 // Inflate obstacle cells by vehicle margins
    void inflateObstacleDynamic(int cx, int cy) {
        int vehicle_cells    = static_cast<int>((vehicle_width_ / 2.0) / resolution_);
        int inflation_radius = std::max(3, vehicle_cells + 2);
        for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
            for (int dy = -inflation_radius; dy <= inflation_radius; dy++)
             {
                int x = cx + dx;
                int y = cy + dy;
                if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_) 
                {
                    int idx = y * map_width_ + x;
                    if (costmap_data_[idx] < 100) {
                        float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
                        int cost = std::max(0, static_cast<int>(80.0f - dist * 10.0f));
                        costmap_data_[idx] = static_cast<int8_t>(
                            std::max(static_cast<int>(costmap_data_[idx]), cost));
                    }
                }
            }
       
        }
    }
 
    double calculateSafeDistance() {
        double reaction_distance   = current_velocity_ * reaction_time_;
        double braking_distance    = (current_velocity_ * current_velocity_) / (2.0 * max_decel_);
        double total_safe_distance = reaction_distance + braking_distance + min_safe_distance_;
        return std::max(min_safe_distance_, total_safe_distance * comfort_factor_);
    }
 
    bool checkSafetyCorridor() {
        float corridor_width = static_cast<float>(vehicle_width_) + 0.4f;
        if (!obstacle_history_.empty()) {
            for (auto& p : obstacle_history_.back()) {
                if (p.x > 0 && p.x < static_cast<float>(calculateSafeDistance())) {
                    if (std::fabs(p.y) < corridor_width / 2.0f)
                        return false;
                }
            }
        }
        return true;
    }
 
    bool checkRearCorridor() {
        float corridor_width = static_cast<float>(vehicle_width_) + 0.4f;
        if (obstacle_history_.empty()) return true;
        for (auto& p : obstacle_history_.back()) {
            if (p.x < 0 && std::fabs(p.x) < static_cast<float>(reverse_clear_dist_)) {
                if (std::fabs(p.y) < corridor_width / 2.0f)
                    return false;
            }
        }
        return true;
    }
 
    void emergencyStop() {
        if (emergency_stop_active_) return;
        emergency_stop_active_ = true;
        drive_state_           = BRAKING;
 
        std_msgs::msg::Bool emergency_msg;
        emergency_msg.data = true;
        emergency_pub_->publish(emergency_msg);
 
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp       = this->now();
        cmd.drive.speed        = 0.0f;
        cmd.drive.acceleration = static_cast<float>(-max_decel_);
        cmd_pub_->publish(cmd);
    }
 
    void safetyMonitor() {
        if (emergency_stop_active_) {
            float front = getRegionDistance("front_center");
            if (front > static_cast<float>(calculateSafeDistance())) {
                emergency_stop_active_ = false;
                drive_state_           = CLEAR;
                std_msgs::msg::Bool emergency_msg;
                emergency_msg.data = false;
                emergency_pub_->publish(emergency_msg);
            }
        }
    }
 
    float getRegionDistance(const std::string& region) {
        auto it = confirmed_obstacles_.find(region);
        if (it != confirmed_obstacles_.end()) return it->second;
        auto it2 = region_min_.find(region);
        if (it2 != region_min_.end()) return it2->second;
        return 999.9f;
    }
 
    void makeVDecision(bool corridor_safe) {
        rclcpp::Time now = this->now();
 
        double safe_dist  = calculateSafeDistance();
        float front       = getRegionDistance("front_center");
        float front_left  = getRegionDistance("front_left");
        float front_right = getRegionDistance("front_right");
        float left        = getRegionDistance("left");
        float right       = getRegionDistance("right");
        float rear        = getRegionDistance("rear");
 
        bool rear_clear = checkRearCorridor();

		 // obstacle in front within stop distance and cant reverse
        bool emergency  = (front < static_cast<float>(min_safe_distance_)) && !rear_clear;
 
        if (emergency) {
            emergencyStop();
            return;
        }
 
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp    = this->now();
        cmd.header.frame_id = "base_link";
 
        std::string status        = "";
        float       desired_speed = 0.0f;
        float       desired_steer = 0.0f;
 
        if (drive_state_ == REVERSING) {
            double elapsed = (now - reverse_start_time_).seconds();
            if (!rear_clear) {    // rear is  blocked mid reverse, stop immediately
                drive_state_  = BRAKING;
                desired_speed = 0.0f;
                desired_steer = 0.0f;
                status        = "reverse stopped: rear blocked";
            } else if (elapsed < reverse_duration_) {
                desired_speed = static_cast<float>(-reverse_speed_);
                desired_steer = recover_steer_dir_ * static_cast<float>(gentle_steering_);
                status        = "REVERSING";
            } else { // reverse complete, transition to frecovery
                drive_state_  = RECOVERING;
                desired_speed = 0.0f;
                desired_steer = 0.0f;
                status        = "transition to RECOVERING";
            }
 
        } else if (drive_state_ == RECOVERING) { // path clear, resume normal driving
            if (front > static_cast<float>(safe_dist)) {
                drive_state_  = CLEAR;
                desired_speed = static_cast<float>(normal_speed_);
                desired_steer = 0.0f;
                status        = "RECOVERED";
            } else {
                desired_speed = static_cast<float>(crawl_speed_);
                desired_steer = recover_steer_dir_ * static_cast<float>(normal_steering_);
                status        = "RECOVERING: steering";
            }
 
        } else {
            if (front <= static_cast<float>(safe_dist) && rear_clear 
            && (now - last_reverse_time_).seconds() > 2.0){
				//  reverse steering direction toward the side with more space
                if (left > right + 0.3f)
                    recover_steer_dir_ = 1.0f;
                else if (right > left + 0.3f)
                    recover_steer_dir_ = -1.0f;
                else
                    recover_steer_dir_ = (front_left >= front_right) ? 1.0f : -1.0f;
 
                drive_state_        = REVERSING;
                last_reverse_time_  = now;
                reverse_start_time_ = now;
                desired_speed       = static_cast<float>(-reverse_speed_);
                desired_steer       = 0.0f;
                status              = "STARTING REVERSE";
 
            } else if (front < static_cast<float>(safe_dist)) {
                desired_speed = 0.0f;
                if (left > right + 0.3f)
                    desired_steer = static_cast<float>(normal_steering_);
                else if (right > left + 0.3f)
                    desired_steer = static_cast<float>(-normal_steering_);
                else
                    desired_steer = 0.0f;
                drive_state_ = BRAKING;
 
                std::ostringstream ss;
                ss << "BRAKING - Front: " << std::fixed << std::setprecision(2)
                   << front << "m < Safe: " << safe_dist << "m";
                status = ss.str();
 
            } else if (front < static_cast<float>(safe_dist * 1.5)) {
                desired_speed = static_cast<float>(slow_speed_);
                if (front_left > front_right + 0.5f)
                    desired_steer = static_cast<float>(gentle_steering_);
                else if (front_right > front_left + 0.5f)
                    desired_steer = static_cast<float>(-gentle_steering_);
                else
                    desired_steer = 0.0f;
                drive_state_ = CAUTION;
 
                std::ostringstream ss;
                ss << "CAUTION - Front: " << std::fixed << std::setprecision(2) << front << "m";
                status = ss.str();
 
            } else if (front > static_cast<float>(safe_dist + 0.2)) {
                desired_speed = static_cast<float>(normal_speed_);
                desired_steer = 0.0f;
                drive_state_  = CLEAR;
 
                std::ostringstream ss;
                ss << "CLEAR - Front: " << std::fixed << std::setprecision(2) << front << "m";
                status = ss.str();
            }
        }
 
        desired_speed = std::max(static_cast<float>(-reverse_speed_),
                                 std::min(desired_speed, static_cast<float>(normal_speed_)));
        desired_steer = std::max(static_cast<float>(-max_steering_angle_),
                                 std::min(desired_steer, static_cast<float>(max_steering_angle_)));
 
        cmd.drive.speed          = desired_speed;
        cmd.drive.steering_angle = desired_steer;
        cmd_pub_->publish(cmd);
 
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = desired_speed;
        twist.angular.z = desired_steer * desired_speed / static_cast<float>(wheelbase_);
        twist_pub_->publish(twist);
 
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "%s | State: %s | V: %.2fm/s | Front: %.2fm | Rear: %.2fm",
            status.c_str(), drive_state_.c_str(), current_velocity_, front, rear);
    }
 
    void publishCostmap() {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = "base_link";
        msg.info.resolution = resolution_;
        msg.info.width      = map_width_;
        msg.info.height     = map_height_;
        msg.info.origin.position.x = -(robot_cell_x_ * resolution_);
        msg.info.origin.position.y = -(robot_cell_y_ * resolution_);
        msg.data.assign(costmap_data_.begin(), costmap_data_.end());
        costmap_pub_->publish(msg);
    }
 
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_; //  for rosbag
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;

	// Publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                        emergency_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr               costmap_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                  twist_pub_;

    rclcpp::TimerBase::SharedPtr costmap_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;

    // Vehicle state
    double current_velocity_;
    double current_steering_;
    bool   emergency_stop_active_;
 
    std::string  drive_state_;
    rclcpp::Time reverse_start_time_;
    rclcpp::Time last_reverse_time_;
    float        recover_steer_dir_;

    // Parameters
    double vehicle_width_;
    double vehicle_length_;
    double max_decel_;
    double reaction_time_;
    double wheelbase_;
    double reverse_speed_;
    double reverse_duration_;
    double reverse_clear_dist_;
    double min_safe_distance_;
    double comfort_factor_;
    double normal_speed_;
    double slow_speed_;
    double crawl_speed_;
    double max_steering_angle_;
    double normal_steering_;
    double gentle_steering_;

    // obstacle tracking
    std::deque<std::vector<ObstaclePoint>> obstacle_history_;
    std::map<std::string, float>           confirmed_obstacles_;
    std::map<std::string, Region>          regions_;
    std::map<std::string, float>           region_min_;
 
    int                 map_width_;
    int                 map_height_;
    float               resolution_;
    int                 robot_cell_x_;
    int                 robot_cell_y_;
    std::vector<int8_t> costmap_data_;
};
 
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidance>();
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        node->shutdownStop();
    }
    rclcpp::shutdown();
    return 0;
}
