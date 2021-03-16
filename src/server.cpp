/* Obstacle server (C++11)
 *
 * Obstacle server is a node fusing data about obstacles from various sources. The idea
 * is to provide reliable and aggregated data to planners.
 * This implementation is split into serveral threads using timers:
 *  - serverPublish() - periodically sending obstacle data to other nodes,
 *  - transformListener() - periodically listening to /tf for transform map->laser.
 */

#include <chrono>

#include "ros/ros.h"
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "obstacle_msgs/ObstaclesStamped.h"

std::map<std::string, std::pair<std_msgs::Header, obstacle_msgs::Obstacles>> m;
ros::Publisher pub_os;
tf::TransformListener* listener;
tf::StampedTransform transform;

ros::Time latest;


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
            this->_start = std::chrono::high_resolution_clock::now();
        }

        void end() {
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
            this->delay(header.stamp);
        }

        void delay(ros::Time stamp) {
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


// Obstacles callback
void osCallback(const ros::MessageEvent<obstacle_msgs::ObstaclesStamped const>& event) {
    // https://github.com/wjwwood/conn_header_test/blob/master/src/listener.cpp
    /*
        const std::string& publisher_name = event.getPublisherName();
        const ros::M_string& header = event.getConnectionHeader();
        ros::Time receipt_time = event.getReceiptTime();

        const obstacle_msgs::ObstaclesStamped::ConstPtr& data = event.getMessage();

        std::cout << "Header:\n";
        for (auto &kv : header) {
            std::cout << "  " << kv.first << " has value " << kv.second << std::endl;
        }
    */
    m[event.getConnectionHeader()["callerid"].c_str()] = std::pair<std_msgs::Header, obstacle_msgs::Obstacles>(event.getMessage()->header, event.getMessage()->obstacles);
    //latest = event.getMessage()->header.stamp;
    //ROS_INFO("%f", m[event.getConnectionHeader()["callerid"].c_str()].segments[0].points[0].x);
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
    msg.header.frame_id = "laser";

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

    for (auto it = m.begin(); it != m.end(); ++it) {
        //std::cout << it->first << std::endl;

        if (std::get<1>(it->second).circles.size() > 0) {
            if (std::get<0>(it->second).frame_id == "laser") {
                msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());
            } else {
                int oldsize = msg.obstacles.circles.size();
                msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());

                double _x, _y;
                for (auto circle = msg.obstacles.circles.begin() + oldsize; circle != msg.obstacles.circles.end(); ++circle) {
                    _x = circle->center.x;
                    _y = circle->center.y;

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
    pub_os.publish(msg);
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

    /* Subscribers
     *
     * Obstacle server should be compatible with various message types, like:
     *  - ObstaclesStamped
     *  - Obstacles (TBI)
     *  - LaserScan
     */
    ros::Subscriber sub_os = n.subscribe("/obstacles_in", 1, osCallback);
    listener = new(tf::TransformListener); // Cannot be global as it leads to "call init first"
    ros::Subscriber sub_ls = n.subscribe("/scan", 1, lsCallback, ros::TransportHints().tcpNoDelay());

    // Publishers
    pub_os = n.advertise<obstacle_msgs::ObstaclesStamped>("/obstacles_out", 1);

    // Timers
    //ros::Timer tim_obstacles = n.createTimer(ros::Duration(0.025), serverPublish);
    ros::Timer tim_transform = n.createTimer(ros::Duration(0.01), transformListener);
    ros::Timer tim_summary = n.createTimer(ros::Duration(2), printSummary);

    // Spin
    ros::spin();

    return 0;
}
