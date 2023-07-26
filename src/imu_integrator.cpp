// To subscribe the IMU topic (50Hz) and Odometry topic (1Hz),
//  and  publish the smooth odometry trajectory (50Hz).

#include "rosbag/imu_integrator.h"

int cnt = 0;
ImuIntegrator::ImuIntegrator(const ros::Publisher &pub)
{
    Eigen::Vector3d zero(0, 0, 0);
    pose.pos = zero;
    pose.orien = Eigen::Matrix3d::Identity();
    velocity = zero;
    firstT = true;
    firstIMU = true;
    firstOdom = true;
    calc_pub = pub;
}

void ImuIntegrator::ImuCallback(const sensor_msgs::Imu &msg)
{

    if (firstT)
    {
        time = msg.header.stamp;
        deltaT = 0;
        setGravity(msg.linear_acceleration);
    }
    else
    {
        if (time > msg.header.stamp)
        {
            return;
        }
        if (firstIMU)
        {
            setGravity(msg.linear_acceleration);
            firstIMU = false;
        }

        deltaT = (msg.header.stamp - time).toSec();
        time = msg.header.stamp;
        calcOrientation(msg.angular_velocity);
        // calcOrientation(msg.orientation);
        calcPosition(msg.linear_acceleration);
        publishMessage(time, pose.pos, pose.orien);
    }
}

void ImuIntegrator::OdomCallback(const nav_msgs::Odometry &msg)
{
    if (firstT)
    {
        firstT = false;
    }
    else if (time > msg.header.stamp)
    {
        return;
    }
    deltaT = (msg.header.stamp - pre_time).toSec();
    pose.pos(0) = msg.pose.pose.position.x;
    pose.pos(1) = msg.pose.pose.position.y;
    pose.pos(2) = msg.pose.pose.position.z;

    Eigen::Quaterniond quat;
    quat.x() = msg.pose.pose.orientation.x;
    quat.y() = msg.pose.pose.orientation.y;
    quat.z() = msg.pose.pose.orientation.z;
    quat.w() = msg.pose.pose.orientation.w;
    pose.orien = quat.normalized().toRotationMatrix();

    if (!firstOdom && imu_R.size())
    {
        velocity = (pose.pos - pre_pos) / deltaT;
        velocity = pose.orien * imu_R[imu_R.size() / 2].inverse() * velocity;
        imu_R.clear();
    }
    if (!firstOdom)
    {
        imu_R.clear();
    }

    pre_pos = pose.pos;
    pre_time = msg.header.stamp;
    firstOdom = false;
    time = msg.header.stamp;
    calc_pub.publish(msg);
}

void ImuIntegrator::setGravity(const geometry_msgs::Vector3 &msg)
{
    gravity[0] = msg.x;
    gravity[1] = msg.y;
    gravity[2] = msg.z;

    std::cout << "Set gravity to:\n"
              << msg << "\n";
}

void ImuIntegrator::publishMessage(const ros::Time time, const Eigen::Vector3d &pos, const Eigen::Matrix3d &orien)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = time;

    odom.pose.pose.position.x = pos(0);
    odom.pose.pose.position.y = pos(1);
    odom.pose.pose.position.z = pos(2);

    odom.pose.pose.orientation.w = std::sqrt(1 + orien(0, 0) + orien(1, 1) + orien(2, 2)) / 2;
    odom.pose.pose.orientation.x = (orien(2, 1) - orien(1, 2)) / 4 / odom.pose.pose.orientation.w;
    odom.pose.pose.orientation.y = (orien(0, 2) - orien(2, 0)) / 4 / odom.pose.pose.orientation.w;
    odom.pose.pose.orientation.z = (orien(1, 0) - orien(0, 1)) / 4 / odom.pose.pose.orientation.w;

    calc_pub.publish(odom);
}

// 通过角加速度推导方向
void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg)
{
    Eigen::Matrix3d B;
    B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT,
        -msg.y * deltaT, msg.x * deltaT, 0;
    double sigma =
        std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
        deltaT;
    pose.orien = pose.orien *
                 (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                  ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);

    imu_R.push_back(pose.orien);
}
// 更新：直接使用IMU返回的方向, q(w,x,y,z)
void ImuIntegrator::calcOrientation(const geometry_msgs::Quaternion &msg)
{
    Eigen::Quaterniond quat;
    quat.x() = msg.x;
    quat.y() = msg.y;
    quat.z() = msg.z;
    quat.w() = msg.w;
    pose.orien = quat.normalized().toRotationMatrix();
    imu_R.push_back(pose.orien);
}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &msg)
{
    Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
    pre_acc = pose.orien * acc_l;

    velocity = velocity + deltaT * (pre_acc - gravity);
    pose.pos = pose.pos + deltaT * velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Imu_Integrator_node");
    ros::NodeHandle nh;
    ros::Publisher calc_pub = nh.advertise<nav_msgs::Odometry>("/calc/odom", 1000);
    ImuIntegrator *imu_integrator = new ImuIntegrator(calc_pub);

    std::cout << "Begin to listen ...\n";

    ros::Subscriber Imu_message = nh.subscribe(
        "/lunar/imu", 2000, &ImuIntegrator::ImuCallback, imu_integrator);
    ros::Subscriber Odom_message = nh.subscribe(
        "/lunar/odom", 2000, &ImuIntegrator::OdomCallback, imu_integrator);

    ros::spin();
}
