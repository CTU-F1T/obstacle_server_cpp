/* Obstacle server (C++11)
 *
 * Obstacle server is a node fusing data about obstacles from various sources. The idea
 * is to provide reliable and aggregated data to planners.
 * This implementation is split into serveral threads using timers:
 *  - serverPublish() - periodically sending obstacle data to other nodes,
 *  - transformListener() - periodically listening to /tf for transform map->laser.
 */

#include <chrono>
#include <thread>
#include <algorithm>

#if ROS1_BUILD
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/LaserScan.h"
#include "obstacle_msgs/ObstaclesStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"

using namespace ros;
using namespace tf2;
using namespace tf2_ros;
using namespace sensor_msgs;
using namespace obstacle_msgs;
using namespace nav_msgs;
//using namespace std_msgs; // In confict with ros::Header

std::map<std::string, std::pair<std_msgs::Header, obstacle_msgs::Obstacles>> m;
ros::Publisher pub_os;
ros::Publisher pub_og;
ros::Publisher pub_ptobs;
std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};
std::unique_ptr<tf2_ros::Buffer> buffer;
geometry_msgs::TransformStamped transform;

ros::Time latest;
#elif ROS2_BUILD
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "obstacle_msgs/msg/obstacles_stamped.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace rclcpp;
using namespace tf2;
using namespace sensor_msgs::msg;
using namespace obstacle_msgs::msg;
using namespace nav_msgs::msg;
using namespace std_msgs::msg;

rclcpp::Node::SharedPtr n;

std::map<std::string, std::pair<std_msgs::msg::Header, obstacle_msgs::msg::Obstacles>> m;
rclcpp::Publisher<obstacle_msgs::msg::ObstaclesStamped>::SharedPtr pub_os;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_og;
rclcpp::Publisher<obstacle_msgs::msg::ObstaclesStamped>::SharedPtr pub_ptobs;
std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};
std::unique_ptr<tf2_ros::Buffer> buffer;
geometry_msgs::msg::TransformStamped transform;

rclcpp::Time latest;
#endif // ROS2_BUILD
std::string ls_frame = "laser";
bool delay_measure = false;
std::string frame_id = "map";
std::string child_frame_id = "base_link";

// Path
bool check_path = false;
#if ROS1_BUILD
Path::ConstPtr reference_path;
#elif ROS2_BUILD
Path::ConstSharedPtr reference_path;
#endif // ROS2_BUILD

// Map Filter
// Removes map from the LaserScan
bool filter_map = false;

// Map
#define FIX_MAP_SIZE 1
#define ROTATE_LOCAL_MAP 1
// All of these are multiplied by 2.
#define INFLATION 6
#define INFLATION_FILTER 2
#define LOCAL_MAP_WIDTH 50
#define LOCAL_MAP_HEIGHT 50
bool map_received = false;
MapMetaData metadata;
std::vector<int8_t> saved_map;


// TimeMeasurer
class TimeMeasurer {
    private:
        std::string _name = "";
        std::string _unit = "";
        std::chrono::time_point<std::chrono::high_resolution_clock> _start;
        int _count = 0;
        double _sum = 0;
        double _max = 0;
        double _last = 0;

        void updateStatistics() {
            this->_count++;
            this->_sum += this->_last;
            this->_max = this->_last > this->_max ? this->_last : this->_max;
        }

    public:
        TimeMeasurer() {
            this->_name = "TimeMeasurer";
            this->_unit = "s";
        };

        TimeMeasurer(std::string name) {
            this->_name = name;
            this->_unit = "s";
        };

        TimeMeasurer(std::string name, std::string unit) {
            this->_name = name;
            this->_unit = unit;
        };

        ~TimeMeasurer(){};

        void start() {
            if (delay_measure)
                this->_start = std::chrono::high_resolution_clock::now();
        }

        void end() {
            if (!delay_measure) return;

            if (this->_unit == "s") {
                this->_last = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - this->_start).count();
            } else if (this->_unit == "ms") {
                this->_last = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - this->_start).count();
            } else if (this->_unit == "us") {
                this->_last = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - this->_start).count();
            } else if (this->_unit == "ns") {
                this->_last = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - this->_start).count();
            } else {
                this->_last = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - this->_start).count();
            }

            this->updateStatistics();
        }

        friend std::ostream& operator<< (std::ostream& os, const TimeMeasurer& tm) {
            //os << boost::format("%s: cur=%.4f%s avg=%.4f%s max=%.4f%s" % tm._name % tm._last % tm->_unit % (tm->_sum / tm->_count) % tm->_unit % tm->_max % tm->_unit;
            if (tm._count > 0) {
                os << tm._name << ": cur=" << tm._last << tm._unit << " avg=" << (tm._sum / tm._count) << tm._unit << " max=" << tm._max << tm._unit;
            } else {
                os << tm._name << ": cur=" << tm._last << tm._unit << " avg=NONE" << " max=" << tm._max << tm._unit;
            }
            return os;
        }
};


// DelayMeasurer
class DelayMeasurer {
    private:
        std::string _name = "";
        std::string _unit = "";
        Time _start;
        int _count = 0;
        double _sum = 0;
        double _max = 0;
        double _last = 0;

        void updateStatistics() {
            this->_count++;
            this->_sum += this->_last;
            this->_max = this->_last > this->_max ? this->_last : this->_max;
        }

    public:
        DelayMeasurer() {
            this->_name = "DelayMeasurer";
            this->_unit = "s";
        };

        DelayMeasurer(std::string name) {
            this->_name = name;
            this->_unit = "s";
        };

        DelayMeasurer(std::string name, std::string unit) {
            this->_name = name;
            this->_unit = unit;
        };

        ~DelayMeasurer(){};

#if ROS1_BUILD
        void delay(std_msgs::Header header) {
#elif ROS2_BUILD
        void delay(Header header) {
#endif // ROS2_BUILD
            if (delay_measure)
                this->delay(header.stamp);
        }

        void delay(Time stamp) {
            if (!delay_measure) return;

#if ROS1_BUILD
            if (this->_unit == "s") {
                this->_last = (ros::Time::now() - stamp).toSec();
            } else if (this->_unit == "ms") {
                this->_last = (ros::Time::now() - stamp).toSec() * 1000;
            } else if (this->_unit == "us") {
                this->_last = (ros::Time::now() - stamp).toSec() * 1000000;
            } else if (this->_unit == "ns") {
                this->_last = (ros::Time::now() - stamp).toNSec();
            } else {
                this->_last = (ros::Time::now() - stamp).toSec();
            }
#elif ROS2_BUILD
            if (this->_unit == "s") {
                this->_last = (n->get_clock()->now() - stamp).seconds();
            } else if (this->_unit == "ms") {
                this->_last = (n->get_clock()->now() - stamp).seconds() * 1000;
            } else if (this->_unit == "us") {
                this->_last = (n->get_clock()->now() - stamp).seconds() * 1000000;
            } else if (this->_unit == "ns") {
                this->_last = (n->get_clock()->now() - stamp).nanoseconds();
            } else {
                this->_last = (n->get_clock()->now() - stamp).seconds();
            }
#endif // ROS2_BUILD

            this->updateStatistics();
        }

        friend std::ostream& operator<< (std::ostream& os, const DelayMeasurer& dm) {
            if (dm._count > 0) {
                os << dm._name << ": cur=" << dm._last << dm._unit << " avg=" << (dm._sum / dm._count) << dm._unit << " max=" << dm._max << dm._unit;
            } else {
                os << dm._name << ": cur=" << dm._last << dm._unit << " avg=NONE" << " max=" << dm._max << dm._unit;
            }
            return os;
        }
};


// Inflation utility
void inflateObstacle(OccupancyGrid* map, int _x, int _y, double radius = 0.0) {
    if ((_y * map->info.width + _x) < map->data.size()) {
        //map.data.at(_y * map.info.width + _x) = 100;

        int inflation = INFLATION;

        if (radius > 0.0) {
            inflation = int(radius / map->info.resolution);
        }

        for (int x = -inflation; x < inflation; x++) {
            if (x + _x < 0 || x + _x >= map->info.width) {
                continue;
            }

            for (int y = -inflation; y < inflation; y++) {
                if (y + _y < 0 || y + _y >= map->info.height) {
                    continue;
                }

                if (x*x + y*y >= inflation * inflation) {
                    continue;
                }

                map->data.at((y + _y) * map->info.width + (x + _x)) = 100;
            }
        }
    }
}


// Filtration utility
bool filterObstacle(OccupancyGrid* map, int _x, int _y, double radius = 0.0) {
    if ((_y * map->info.width + _x) < map->data.size()) {
        //map.data.at(_y * map.info.width + _x) = 100;

        int inflation = INFLATION_FILTER;

        if (radius > 0.0) {
            inflation = int(radius / map->info.resolution);
        }

        for (int x = -inflation; x < inflation; x++) {
            if (x + _x < 0 || x + _x >= map->info.width) {
                continue;
            }

            for (int y = -inflation; y < inflation; y++) {
                if (y + _y < 0 || y + _y >= map->info.height) {
                    continue;
                }

                if (x*x + y*y >= inflation * inflation) {
                    continue;
                }

                if (map->data.at((y + _y) * map->info.width + (x + _x)) == 100) {
                    return true;
                }
            }
        }
    }

    return false;
}


// Obstacles callback
#if ROS1_BUILD
void osCallback(const ros::MessageEvent<obstacle_msgs::ObstaclesStamped const>& event) {
    m[event.getConnectionHeader()["callerid"].c_str()] = std::pair<std_msgs::Header, obstacle_msgs::Obstacles>(event.getMessage()->header, event.getMessage()->obstacles);
}
#elif ROS2_BUILD
void osCallback(obstacle_msgs::msg::ObstaclesStamped::ConstSharedPtr message) {
    m["obstacles"] = std::pair<std_msgs::msg::Header, obstacle_msgs::msg::Obstacles>(message->header, message->obstacles);
}
#endif


// Path callback
#if ROS1_BUILD
void ptCallback(const ros::MessageEvent<nav_msgs::Path const>& event) {
    nav_msgs::Path::ConstPtr message = event.getMessage();
#elif ROS2_BUILD
void ptCallback(nav_msgs::msg::Path::ConstSharedPtr message) {
#endif // ROS2_BUILD
    reference_path = message;
    check_path = true;
}


// OccupancyGrid callback
#if ROS1_BUILD
void ogCallback(const ros::MessageEvent<nav_msgs::OccupancyGrid const>& event) {
    nav_msgs::OccupancyGrid::ConstPtr message = event.getMessage();
#elif ROS2_BUILD
void ogCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
#endif // ROS2_BUILD
    //saved_map = message;
    metadata = message->info;
    saved_map = message->data;

    for (int i = 0; i < metadata.width; i++) {
        for (int j = 0; j < metadata.height; j++) {
            if (message->data.at(j * metadata.width + i) > 50 && message->data.at(j * metadata.width + i) <= 100) {
                for (int x = -INFLATION; x < INFLATION; x++) {
                    if (x + i < 0 || x + i >= metadata.width) {
                        continue;
                    }

                    for (int y = -INFLATION; y < INFLATION; y++) {
                        if (y + j < 0 || y + j >= metadata.height) {
                            continue;
                        }

                        if (x*x + y*y >= INFLATION * INFLATION) {
                            continue;
                        }

                        auto current = saved_map.at((j + y) * metadata.width + x + i);

                        if (current < message->data.at(j * metadata.width + i)) {
                            // We set this to 100, to make it clear we are dealing with an obstacle.
                            saved_map.at((j + y) * metadata.width + x + i) = 100;
                        }
                    }
                }
            }
        }
    }
}


void serverPublish();

TimeMeasurer tm_laserscan("laserscan", "us");
DelayMeasurer dm_laserscanstart("scandelay_start", "ms");
DelayMeasurer dm_laserscan("scandelay", "ms");

// LaserScan callback
#if ROS1_BUILD
void lsCallback(const ros::MessageEvent<sensor_msgs::LaserScan const>& event) {
    sensor_msgs::LaserScan::ConstPtr message = event.getMessage();
#elif ROS2_BUILD
void lsCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
#endif // ROS2_BUILD
    dm_laserscanstart.delay(message->header.stamp);
    tm_laserscan.start();

    std::shared_ptr<ObstaclesStamped> os(new ObstaclesStamped);
    //std::vector<CircleObstacle> obs(message->ranges.size());
    os->obstacles.circles.reserve(message->ranges.size());


    // Convert LaserScan to CircleObstacle[]
    // TODO: Use std::transform
    int i;
    for (float angle = message->angle_min, i = 0; i < message->ranges.size(); angle += message->angle_increment, i++) {
        CircleObstacle circle;
        circle.center.x = cos(angle) * message->ranges.at(i);
        circle.center.y = sin(angle) * message->ranges.at(i);
        circle.radius = 0.01;
        os->obstacles.circles.emplace_back(circle);
    }

    // Remember frame id
    ls_frame = message->header.frame_id;

    // Store scan
#if ROS1_BUILD
    m[event.getConnectionHeader()["callerid"].c_str()] = std::pair<std_msgs::Header, obstacle_msgs::Obstacles>(event.getMessage()->header, os->obstacles);
#elif ROS2_BUILD
    m["scan"] = std::pair<std_msgs::msg::Header, obstacle_msgs::msg::Obstacles>(message->header, os->obstacles);
#endif
    latest = message->header.stamp;

    tm_laserscan.end();
    dm_laserscan.delay(message->header.stamp);

    serverPublish();
}


TimeMeasurer tm_server("server", "us");
DelayMeasurer dm_serverstart("serverdelay_start", "ms");
DelayMeasurer dm_server("serverdelay", "ms");

// Periodic publisher
//void serverPublish(const ros::TimerEvent&) {
void serverPublish() {
#if ROS1_BUILD
    if (m.size() > 0 and latest.toSec() > 0) {
#elif ROS2_BUILD
    if (m.size() > 0 and latest.seconds() > 0) {
#endif // ROS2_BUILD
        dm_serverstart.delay(latest);
    }
    tm_server.start();
    //std::cout << "Currently logged sources: " << m.size() << std::endl;

    ObstaclesStamped msg;
#if ROS1_BUILD
    msg.header.stamp = ros::Time::now();
#elif ROS2_BUILD
    msg.header.stamp = n->get_clock()->now();
#endif // ROS2_BUILD
    msg.header.frame_id = ls_frame;

    // Hold onto transformation
    // It is better to do it here as we don't use every transformation.
    tf2::Stamped<tf2::Transform> _transform;
    tf2::fromMsg(transform, _transform);

    double roll, pitch, yaw, ysin, ycos, tx, ty;
    Vector3 vec = _transform.getOrigin();
    tx = vec.getX();
    ty = vec.getY();
    Matrix3x3(Quaternion(_transform.getRotation())).getRPY(roll, pitch, yaw);
    ysin = sin(yaw);
    ycos = cos(yaw);

    // Inverse transformation
    double rollI, pitchI, yawI, ysinI, ycosI, txI, tyI;
    Vector3 vecI = _transform.inverse().getOrigin();
    txI = vecI.getX();
    tyI = vecI.getY();
    Matrix3x3(Quaternion(_transform.inverse().getRotation())).getRPY(rollI, pitchI, yawI);
    ysinI = sin(yawI);
    ycosI = cos(yawI);

    // OccupancyGrid
    OccupancyGrid map;
#if ROS2_BUILD
    map.header.stamp = n->get_clock()->now();
#endif // ROS2_BUILD
    map.header.frame_id = "map";
    map.info = metadata;
    map.data = saved_map;

    for (auto it = m.begin(); it != m.end(); ++it) {
        //std::cout << it->first << std::endl;

        if (std::get<1>(it->second).circles.size() > 0) {
            if (std::get<0>(it->second).frame_id == ls_frame) {
                if (!filter_map) {
                    msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());
                } else {
                    for (auto circle = std::get<1>(it->second).circles.begin(); circle != std::get<1>(it->second).circles.end(); ++circle) {
                        if (!filterObstacle(&map, int((circle->center.x * ycosI - circle->center.y * ysinI + txI - map.info.origin.position.x) / map.info.resolution), int((circle->center.x * ysinI + circle->center.y * ycosI + tyI - map.info.origin.position.y) / map.info.resolution))) {
                            msg.obstacles.circles.push_back(*circle);
                        }
                    }
                }

                for (auto circle = std::get<1>(it->second).circles.begin(); circle != std::get<1>(it->second).circles.end(); ++circle) {
                    inflateObstacle(&map,
                        int((circle->center.x * ycosI - circle->center.y * ysinI + txI - map.info.origin.position.x) / map.info.resolution),
                        int((circle->center.x * ysinI + circle->center.y * ycosI + tyI - map.info.origin.position.y) / map.info.resolution)
                    );
                }

            } else {
                int oldsize = msg.obstacles.circles.size();
                // TODO: Consider whether to filter obstacles from the map as well.
                msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());

                double _x, _y;
                for (auto circle = msg.obstacles.circles.begin() + oldsize; circle != msg.obstacles.circles.end(); ++circle) {
                    _x = circle->center.x;
                    _y = circle->center.y;

                    inflateObstacle(&map,
                        int((_x - map.info.origin.position.x) / map.info.resolution),
                        int((_y - map.info.origin.position.y) / map.info.resolution),
                    circle->radius);

                    // TODO: What is the purpose of this?
                    circle->center.x = _x * ycos - _y * ysin + tx;
                    circle->center.y = _x * ysin + _y * ycos + ty;
                }
            }
        }
    }

    tm_server.end();
#if ROS1_BUILD
    if (m.size() > 0 and latest.toSec() > 0) {
#elif ROS2_BUILD
    if (m.size() > 0 and latest.seconds() > 0) {
#endif // ROS2_BUILD
        dm_server.delay(latest);
    }


    // Find overlap with given Path (if obtained)
    if (check_path) {
        ObstaclesStamped msg_path;
#if ROS1_BUILD
        msg_path.header.stamp = ros::Time::now();
#elif ROS2_BUILD
        msg_path.header.stamp = n->get_clock()->now();
#endif // ROS2_BUILD
        msg_path.header.frame_id = "map";
        for (auto p : reference_path->poses) {
            if (map.data.at(
                        int((p.pose.position.x - map.info.origin.position.x) / map.info.resolution) +
                        int((p.pose.position.y - map.info.origin.position.y) / map.info.resolution) * map.info.width
                ) == 100) {
                    CircleObstacle circle;
                    circle.center.x = p.pose.position.x;
                    circle.center.y = p.pose.position.y;
                    circle.radius = map.info.resolution / 2.0;
                    msg_path.obstacles.circles.emplace_back(circle);
            }
        }
        pub_ptobs.publish(msg_path);
    }


    // Cut the map
    std::vector<int8_t> new_map;

    auto car_x = int((txI - map.info.origin.position.x) / map.info.resolution);
    auto car_y = int((tyI - map.info.origin.position.y) / map.info.resolution);

    auto real_width = std::min(int(map.info.width), car_x+LOCAL_MAP_WIDTH) - std::max(0, car_x - LOCAL_MAP_WIDTH);
    auto real_height = std::min(int(map.info.height), car_y + LOCAL_MAP_HEIGHT) - std::max(0, car_y - LOCAL_MAP_HEIGHT);

#if FIX_MAP_SIZE
    new_map.resize(4 * LOCAL_MAP_WIDTH * LOCAL_MAP_HEIGHT, 255);

    int iter = std::max(0, LOCAL_MAP_HEIGHT - car_y);
    for (int i = std::max(0, car_y - LOCAL_MAP_HEIGHT); i < std::min(int(map.info.height), car_y + LOCAL_MAP_HEIGHT); i++, iter++) {

        //new_map.insert(new_map.end(), map.info.begin() + (i * map.info.width),
        memcpy(&new_map[iter*2*LOCAL_MAP_WIDTH + std::max(0, LOCAL_MAP_WIDTH - car_x)], &map.data[i * map.info.width + std::max(0, car_x - LOCAL_MAP_WIDTH)], real_width*sizeof(int8_t));
    }

    map.info.origin.position.x += (car_x - LOCAL_MAP_WIDTH) * map.info.resolution;
    map.info.origin.position.y += (car_y - LOCAL_MAP_HEIGHT) * map.info.resolution;
    map.info.width = 2 * LOCAL_MAP_WIDTH;
    map.info.height = 2 * LOCAL_MAP_HEIGHT;
#else
    new_map.resize(4 * (real_width) * (real_height));

    int iter = 0;
    for (int i = std::max(0, car_y - LOCAL_MAP_HEIGHT); i < std::min(int(map.info.height), car_y + LOCAL_MAP_HEIGHT); i++, iter++) {
        //new_map.insert(new_map.end(), map.info.begin() + (i * map.info.width),
        memcpy(&new_map[iter*real_width], &map.data[i * map.info.width + std::max(0, car_x - LOCAL_MAP_WIDTH)], real_width*sizeof(int8_t));
    }

    map.info.origin.position.x += (std::max(0, car_x - LOCAL_MAP_WIDTH) * map.info.resolution);
    map.info.origin.position.y += (std::max(0, car_y - LOCAL_MAP_HEIGHT) * map.info.resolution);
    map.info.width = real_width;
    map.info.height = real_height;
#endif
#if ROTATE_LOCAL_MAP
    // This keeps the origin at the car's position, but rotates the map in such a way, that x-axis is in the direction of the car.
    Quaternion q = Quaternion(_transform.getRotation());
    map.info.origin.orientation.x = q.x();
    map.info.origin.orientation.y = q.y();
    map.info.origin.orientation.z = q.z();
    map.info.origin.orientation.w = q.w();
    Vector3 v = Vector3((LOCAL_MAP_WIDTH) * map.info.resolution, (LOCAL_MAP_HEIGHT) * map.info.resolution, 0);
    Vector3 rv = quatRotate(q, v) - v;
    map.info.origin.position.x -= rv.getX();
    map.info.origin.position.y -= rv.getY();
#endif
    map.data = new_map;

#if ROS1_BUILD
    pub_os.publish(msg);
    pub_og.publish(map);
#elif ROS2_BUILD
    pub_os->publish(msg);
    pub_og->publish(map);
#endif // ROS2_BUILD
}

// Periodic listener
#if ROS1_BUILD
void transformListener(const ros::TimerEvent&) {
    try {
        transform = buffer->lookupTransform(child_frame_id, frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}
#elif ROS2_BUILD
void transformListener() {
    try {
        transform = buffer->lookupTransform(child_frame_id, frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("obstacle_server_cpp"), "%s", ex.what());
    }
}
#endif // ROS2_BUILD


// Periodically print summary
#if ROS1_BUILD
void printSummary(const ros::TimerEvent&) {
#elif ROS2_BUILD
void printSummary() {
#endif // ROS2_BUILD
    std::cout << tm_laserscan << std::endl;
    std::cout << tm_server << std::endl;
    std::cout << dm_laserscanstart << std::endl;
    std::cout << dm_laserscan << std::endl;
    std::cout << dm_serverstart << std::endl;
    std::cout << dm_server << std::endl << std::endl;
}


// Main function
#if ROS1_BUILD
int main(int argc, char **argv) {
    // Initialization and passing of remappings
    ros::init(argc, argv, "obstacle_server_cpp");

    // NodeHandle -- access point to communications within ROS
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    /* Obtain parameters
     *
     * /delay_measure -- when true, timing is performed
     * /filter_map -- when true, map is removed from the LaserScan data
     */
    n.getParam("delay_measure", delay_measure);
    n_private.getParam("filter_map", filter_map);
    n_private.getParam("frame_id", frame_id);
    n_private.getParam("child_frame_id", child_frame_id);

    /* Simulated time workaround
     *
     * When using simulated time (e.g. Stage), Time::now() returns
     * value 0 until something was received from '/clock'. So, we
     * will wait for it.
     *
     * Side note: On the other hand, when running Stage, sometimes
     * the messages arrive from the future. :(
     */
    while (ros::ok()) {
        if ( ros::Time::now().toSec() == 0 ) {
            ROS_WARN("Current time is 0. Waiting for it to change.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
            break;
        }
    }

    /* Subscribers
     *
     * Obstacle server should be compatible with various message types, like:
     *  - ObstaclesStamped
     *  - Obstacles (TBI)
     *  - LaserScan
     *
     * In addition we want to obtain and publish OccupancyGrid with all the obstacles.
     */
    buffer = std::make_unique<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer); // Cannot be global as it leads to "call init first"

    // Wait for transform.
    ROS_WARN_STREAM("Waiting for '" << frame_id << "' -> '" << child_frame_id << "' transform...");
    while (ros::ok()) {
        try {
            transform = buffer->lookupTransform(child_frame_id, frame_id, ros::Time(0));
            ROS_WARN("Transform found.");
            break;
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.01).sleep();
        }
    }

    ros::Subscriber sub_os = n.subscribe("/obstacles_in", 1, osCallback);
    ros::Subscriber sub_ls = n.subscribe("/scan", 1, lsCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_og = n.subscribe("/map_static", 1, ogCallback);
    ros::Subscriber sub_pt = n.subscribe("/path", 1, ptCallback);

    // Publishers
    pub_os = n.advertise<obstacle_msgs::ObstaclesStamped>("/obstacles_out", 1);
    pub_og = n.advertise<nav_msgs::OccupancyGrid>("/map/local", 1);
    pub_ptobs = n.advertise<obstacle_msgs::ObstaclesStamped>("/path/obstacles", 1);

    // Timers
    //ros::Timer tim_obstacles = n.createTimer(ros::Duration(0.025), serverPublish);
    ros::Timer tim_transform = n.createTimer(ros::Duration(0.1), transformListener);
    ros::Timer tim_summary;
    if (delay_measure) tim_summary = n.createTimer(ros::Duration(2), printSummary);

    // Spin
    ros::spin();

    return 0;
}
#elif ROS2_BUILD
int main(int argc, char **argv) {
    // Initialization and passing of remappings
    rclcpp::init(argc, argv);

    // NodeHandle -- access point to communications within ROS
    n = rclcpp::Node::make_shared("obstacle_server_cpp");

    /* Obtain parameters
     *
     * /delay_measure -- when true, timing is performed
     * /filter_map -- when true, map is removed from the LaserScan data
     */
    n->declare_parameter("delay_measure", ParameterValue(false));
    n->declare_parameter("filter_map", ParameterValue(false));
    n->declare_parameter("frame_id", ParameterValue(frame_id));
    n->declare_parameter("child_frame_id", ParameterValue(child_frame_id));

    delay_measure = n->get_parameter("delay_measure").as_bool();
    filter_map = n->get_parameter("filter_map").as_bool();
    frame_id = n->get_parameter("frame_id").as_string();
    child_frame_id = n->get_parameter("child_frame_id").as_string();

    /* Simulated time workaround
     *
     * When using simulated time (e.g. Stage), Time::now() returns
     * value 0 until something was received from '/clock'. So, we
     * will wait for it.
     *
     * Side note: On the other hand, when running Stage, sometimes
     * the messages arrive from the future. :(
     */
    while (rclcpp::ok()) {
        if ( n->get_clock()->now().nanoseconds() == 0 ) {
            RCLCPP_WARN(n->get_logger(), "Current time is 0. Waiting for it to change.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
            break;
        }
    }

    /* Subscribers
     *
     * Obstacle server should be compatible with various message types, like:
     *  - ObstaclesStamped
     *  - Obstacles (TBI)
     *  - LaserScan
     *
     * In addition we want to obtain and publish OccupancyGrid with all the obstacles.
     */

    buffer = std::make_unique<tf2_ros::Buffer>(n->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer); // Cannot be global as it leads to "call init first"

    // Wait for transform.
    RCLCPP_WARN_STREAM(rclcpp::get_logger("obstacle_server_cpp"), "Waiting for '" << frame_id << "' -> '" << child_frame_id << "' transform...");
    while (rclcpp::ok()) {
        try {
            transform = buffer->lookupTransform(child_frame_id, frame_id, tf2::TimePointZero);
            RCLCPP_WARN(rclcpp::get_logger("obstacle_server_cpp"), "Transform found.");
            break;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(rclcpp::get_logger("obstacle_server_cpp"), "%s", ex.what());
            rclcpp::sleep_for(std::chrono::nanoseconds(100000000));
        }
    }

    auto sub_os = n->create_subscription<obstacle_msgs::msg::ObstaclesStamped>("/obstacles_in", rclcpp::QoS(5).best_effort().durability_volatile(), osCallback);
    auto sub_ls = n->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(1).best_effort().durability_volatile(), lsCallback);//, rclcpp::TransportHints().tcpNoDelay());
    auto sub_og = n->create_subscription<nav_msgs::msg::OccupancyGrid>("/map_static", rclcpp::QoS(1).reliable().transient_local(), ogCallback);
    auto sub_pt = n->create_subscription<nav_msgs::msg::Path>("/path", rclcpp::QoS(1).reliable().transient_local(), ptCallback);

    // Publishers
    pub_os = n->create_publisher<obstacle_msgs::msg::ObstaclesStamped>("/obstacles_out", rclcpp::QoS(1).best_effort().durability_volatile());
    pub_og = n->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/local", rclcpp::QoS(1).best_effort().durability_volatile());
    pub_ptobs = n->create_publisher<obstacle_msgs::msg::ObstaclesStamped>("/path/obstacles", rclcpp::QoS(1).best_effort().durability_volatile());

    // Timers
    //rclcpp::Timer tim_obstacles = n.createTimer(rclcpp::Duration(0.025), serverPublish);
    rclcpp::TimerBase::SharedPtr tim_transform {nullptr};
    // Call transformListener function every 10ms
    //tim_transform = n->create_wall_timer(10ms, transformListener);
    tim_transform = create_timer(n, n->get_clock(), rclcpp::Duration::from_seconds(0.01), &transformListener);

    //rclcpp::Timer tim_summary;
    rclcpp::TimerBase::SharedPtr tim_summary {nullptr};
    // Call printSummary function every other second
    if (delay_measure) {
        tim_summary = create_timer(n, n->get_clock(), rclcpp::Duration::from_seconds(2.0), &printSummary);
    }

    // Spin
    rclcpp::spin(n);
    rclcpp::shutdown();

    return 0;
}
#endif // ROS2_BUILD
