#include "ros/ros.h"

#include "obstacle_msgs/ObstaclesStamped.h"

std::map<std::string, std::pair<std_msgs::Header, obstacle_msgs::Obstacles>> m;
ros::Publisher pub_os;

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
    //ROS_INFO("%f", m[event.getConnectionHeader()["callerid"].c_str()].segments[0].points[0].x);
}


// Periodic publisher
void serverPublish(const ros::TimerEvent&) {
    std::cout << "Currently logged sources: " << m.size() << std::endl;

    obstacle_msgs::ObstaclesStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";

    for (auto it = m.begin(); it != m.end(); ++it) {
        std::cout << it->first << std::endl;

        msg.obstacles.circles.insert(msg.obstacles.circles.end(), std::get<1>(it->second).circles.begin(), std::get<1>(it->second).circles.end());
    }

    pub_os.publish(msg);
}


// Main function
int main(int argc, char **argv) {
    // Initialization and passing of remappings
    ros::init(argc, argv, "obstacle_server_cpp");

    // NodeHandle -- access point to communications within ROS
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber sub_os = n.subscribe("/obstacles_in", 1, osCallback);

    // Publishers
    pub_os = n.advertise<obstacle_msgs::ObstaclesStamped>("/obstacles_out", 1);

    // Timers
    ros::Timer tim_obstacles = n.createTimer(ros::Duration(1), serverPublish);

    // Spin
    ros::spin();

    return 0;
}
