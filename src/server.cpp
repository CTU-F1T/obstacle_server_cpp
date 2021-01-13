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

ros::Time latest = ros::Time(0);

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
    latest = event.getMessage()->header.stamp;
    //ROS_INFO("%f", m[event.getConnectionHeader()["callerid"].c_str()].segments[0].points[0].x);
}

double scan_highest = 0;

int scan_mes = 0;
double scan_hig = 0;
double scan_avr = 0;


// LaserScan callback
void lsCallback(const ros::MessageEvent<sensor_msgs::LaserScan const>& event) {
    auto start = std::chrono::high_resolution_clock::now();
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

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
    scan_highest = scan_highest < duration.count() ? duration.count() : scan_highest;

    std::cout << "Scan: " << duration.count() << " , " << scan_highest << std::endl;

    scan_mes++;
    double res = (ros::Time::now() - latest).toSec() * 1000;
    scan_hig = res > scan_hig ? res : scan_hig;
    scan_avr += res;

    std::cout << "S: " << res << "ms (" << scan_hig << "ms/" << scan_avr / scan_mes << "ms)" << std::endl;
}


double highest = 0;

int tim_mes = 0;
double tim_hig = 0;
double tim_avr = 0;

// Periodic publisher
void serverPublish(const ros::TimerEvent&) {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Currently logged sources: " << m.size() << std::endl;

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
        std::cout << it->first << std::endl;

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

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
    highest = highest < duration.count() ? duration.count() : highest;

    std::cout << duration.count() << " , " << highest << std::endl;

    tim_mes++;
    double res = (ros::Time::now() - latest).toSec() * 1000;
    tim_hig = res > tim_hig ? res : tim_hig;
    tim_avr += res;

    std::cout << "O: " << res << "ms (" << tim_hig << "ms/" << tim_avr / tim_mes << "ms)" << std::endl;

    pub_os.publish(msg);
}

// Periodic listener
void transformListener(const ros::TimerEvent&) {
    try {
        listener->lookupTransform("/base_link", "/map", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
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
    ros::Subscriber sub_ls = n.subscribe("/scan", 1, lsCallback);

    // Publishers
    pub_os = n.advertise<obstacle_msgs::ObstaclesStamped>("/obstacles_out", 1);

    // Timers
    ros::Timer tim_obstacles = n.createTimer(ros::Duration(0.05), serverPublish);
    ros::Timer tim_transform = n.createTimer(ros::Duration(0.02), transformListener);

    // Spin
    ros::spin();

    return 0;
}
