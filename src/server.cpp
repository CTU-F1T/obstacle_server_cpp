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

#include "ros/ros.h"
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "obstacle_msgs/ObstaclesStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

std::map<std::string, std::pair<std_msgs::Header, obstacle_msgs::Obstacles>> m;
ros::Publisher pub_os;
ros::Publisher pub_og;
tf::TransformListener* listener;
tf::StampedTransform transform;

ros::Time latest;
std::string ls_frame = "laser";
bool delay_measure = false;

// Map
#define FIX_MAP_SIZE 1
// All of these are multiplied by 2.
#define INFLATION 6
#define LOCAL_MAP_WIDTH 50
#define LOCAL_MAP_HEIGHT 50
bool map_received = false;
nav_msgs::MapMetaData metadata;
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
        ros::Time _start;
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

        void delay(std_msgs::Header header) {
            if (delay_measure)
                this->delay(header.stamp);
        }

        void delay(ros::Time stamp) {
            if (!delay_measure) return;

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
void inflateObstacle(nav_msgs::OccupancyGrid* map, int _x, int _y, double radius = 0.0) {
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


// Obstacles callback
void osCallback(const ros::MessageEvent<obstacle_msgs::ObstaclesStamped const>& event) {
    m[event.getConnectionHeader()["callerid"].c_str()] = std::pair<std_msgs::Header, obstacle_msgs::Obstacles>(event.getMessage()->header, event.getMessage()->obstacles);
}

// OccupancyGrid callback
void ogCallback(const ros::MessageEvent<nav_msgs::OccupancyGrid const>& event) {
    //saved_map = event.getMessage();
    metadata = event.getMessage()->info;
    saved_map = event.getMessage()->data;

    for (int i = 0; i < metadata.width; i++) {
        for (int j = 0; j < metadata.height; j++) {
            if (event.getMessage()->data.at(j * metadata.width + i) > 50 && event.getMessage()->data.at(j * metadata.width + i) <= 100) {
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

                        if (current < event.getMessage()->data.at(j * metadata.width + i)) {
                            saved_map.at((j + y) * metadata.width + x + i) = event.getMessage()->data.at(j * metadata.width + i);
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
void lsCallback(const ros::MessageEvent<sensor_msgs::LaserScan const>& event) {
    dm_laserscanstart.delay(event.getMessage()->header.stamp);
    tm_laserscan.start();
    boost::shared_ptr<obstacle_msgs::ObstaclesStamped> os(new obstacle_msgs::ObstaclesStamped);
    //std::vector<obstacle_msgs::CircleObstacle> obs(event.getMessage()->ranges.size());
    os->obstacles.circles.reserve(event.getMessage()->ranges.size());

    // Convert LaserScan to CircleObstacle[]
    // TODO: Use std::transform
    for (float angle = event.getMessage()->angle_min, i = 0; i < event.getMessage()->ranges.size(); angle += event.getMessage()->angle_increment, i++) {
        obstacle_msgs::CircleObstacle circle;
        circle.center.x = cos(angle) * event.getMessage()->ranges.at(i);
        circle.center.y = sin(angle) * event.getMessage()->ranges.at(i);
        circle.radius = 0.01;
        os->obstacles.circles.emplace_back(circle);
    }

    // Remember frame id
    ls_frame = event.getMessage()->header.frame_id;

    // Store scan
    m[event.getConnectionHeader()["callerid"].c_str()] = std::pair<std_msgs::Header, obstacle_msgs::Obstacles>(event.getMessage()->header, os->obstacles);
    latest = event.getMessage()->header.stamp;

    tm_laserscan.end();
    dm_laserscan.delay(event.getMessage()->header.stamp);

    serverPublish();
}


TimeMeasurer tm_server("server", "us");
DelayMeasurer dm_serverstart("serverdelay_start", "ms");
DelayMeasurer dm_server("serverdelay", "ms");

// Periodic publisher
//void serverPublish(const ros::TimerEvent&) {
void serverPublish() {
    if (m.size() > 0 and latest.toSec() > 0) {
        dm_serverstart.delay(latest);
    }
    tm_server.start();
    //std::cout << "Currently logged sources: " << m.size() << std::endl;

    obstacle_msgs::ObstaclesStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = ls_frame;

    // Hold onto transformation
    // It is better to do it here as we don't use every transformation.
    tf::StampedTransform _transform(transform);

    double roll, pitch, yaw, ysin, ycos, tx, ty;
    tf::Vector3 vec = _transform.getOrigin();
    tx = vec.getX();
    ty = vec.getY();
    tf::Matrix3x3(tf::Quaternion(_transform.getRotation())).getRPY(roll, pitch, yaw);
    ysin = sin(yaw);
    ycos = cos(yaw);

    // Inverse transformation
    double rollI, pitchI, yawI, ysinI, ycosI, txI, tyI;
    tf::Vector3 vecI = _transform.inverse().getOrigin();
    txI = vecI.getX();
    tyI = vecI.getY();
    tf::Matrix3x3(tf::Quaternion(_transform.inverse().getRotation())).getRPY(rollI, pitchI, yawI);
    ysinI = sin(yawI);
    ycosI = cos(yawI);

    // OccupancyGrid
    nav_msgs::OccupancyGrid map;
    map.info = metadata;
    map.data = saved_map;

    for (auto it = m.begin(); it != m.end(); ++it) {
        //std::cout << it->first << std::endl;

        if (std::get<1>(it->second).circles.size() > 0) {
            if (std::get<0>(it->second).frame_id == ls_frame) {
                msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());

                for (auto circle = std::get<1>(it->second).circles.begin(); circle != std::get<1>(it->second).circles.end(); ++circle) {
                    inflateObstacle(&map,
                        int((circle->center.x * ycosI - circle->center.y * ysinI + txI - map.info.origin.position.x) / map.info.resolution),
                        int((circle->center.x * ysinI + circle->center.y * ycosI + tyI - map.info.origin.position.y) / map.info.resolution)
                    );
                }

            } else {
                int oldsize = msg.obstacles.circles.size();
                msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());

                double _x, _y;
                for (auto circle = msg.obstacles.circles.begin() + oldsize; circle != msg.obstacles.circles.end(); ++circle) {
                    _x = circle->center.x;
                    _y = circle->center.y;

                    inflateObstacle(&map, _x, _y, circle->radius);

                    circle->center.x = _x * ycos - _y * ysin + tx;
                    circle->center.y = _x * ysin + _y * ycos + ty;
                }
            }
        }
    }

    tm_server.end();
    if (m.size() > 0 and latest.toSec() > 0) {
        dm_server.delay(latest);
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
    map.data = new_map;

    pub_os.publish(msg);
    pub_og.publish(map);
}

// Periodic listener
void transformListener(const ros::TimerEvent&) {
    try {
        listener->lookupTransform("/base_link", "/map", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        //ROS_ERROR("%s", ex.what());
    }
}


// Periodically print summary
void printSummary(const ros::TimerEvent&) {
    std::cout << tm_laserscan << std::endl;
    std::cout << tm_server << std::endl;
    std::cout << dm_laserscanstart << std::endl;
    std::cout << dm_laserscan << std::endl;
    std::cout << dm_serverstart << std::endl;
    std::cout << dm_server << std::endl << std::endl;
}


// Main function
int main(int argc, char **argv) {
    // Initialization and passing of remappings
    ros::init(argc, argv, "obstacle_server_cpp");

    // NodeHandle -- access point to communications within ROS
    ros::NodeHandle n;

    /* Obtain parameters
     *
     * /delay_measure -- when true, timing is performed
     */
    n.getParam("delay_measure", delay_measure);

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
    ros::Subscriber sub_os = n.subscribe("/obstacles_in", 1, osCallback);
    listener = new(tf::TransformListener); // Cannot be global as it leads to "call init first"
    ros::Subscriber sub_ls = n.subscribe("/scan", 1, lsCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_og = n.subscribe("/map_static", 1, ogCallback);

    // Publishers
    pub_os = n.advertise<obstacle_msgs::ObstaclesStamped>("/obstacles_out", 1);
    pub_og = n.advertise<nav_msgs::OccupancyGrid>("/map/local", 1);

    // Timers
    //ros::Timer tim_obstacles = n.createTimer(ros::Duration(0.025), serverPublish);
    ros::Timer tim_transform = n.createTimer(ros::Duration(0.01), transformListener);
    ros::Timer tim_summary;
    if (delay_measure) tim_summary = n.createTimer(ros::Duration(2), printSummary);

    // Spin
    ros::spin();

    return 0;
}
