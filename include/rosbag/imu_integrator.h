#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/Marker.h>
#include<sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <typeinfo>
#include <vector>
// Custom message includes. Auto-generated from msg/ directory.
/*
struct Orientation
{

};

struct Position
{
    double x, y, z;
};
*/
struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    ros::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d pre_acc;
    Eigen::Vector3d velocity;
    Eigen::Vector3d pre_pos;
    ros::Time pre_time;
    visualization_msgs::Marker path;
    ros::Publisher calc_pub;
    double deltaT;
    bool firstT;
    bool firstIMU;
    bool firstOdom;
    std::vector<Eigen::Matrix3d> imu_R;
public:
    //! Constructor.
    ImuIntegrator(const ros::Publisher &line);
    ImuIntegrator(const ros::Publisher &line, const ros::Publisher &odom_line);
    //! Destructor.
    ~ImuIntegrator();

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage();
    void publishMessage(const ros::Time time, const Eigen::Vector3d &pos, const Eigen::Matrix3d &orien);


    //! Callback function for subscriber.
    void ImuCallback(const sensor_msgs::Imu &msg);
    void OdomCallback(const nav_msgs::Odometry &msg);

    void setGravity(const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Quaternion &msg);

};

#endif // SR_NODE_EXAMPLE_CORE_H
