/*
 *tool centroid of mass identification
 *Created by Gao Ziqi @SIA,CAS/UCAS on 2022/07.
 *Reference:侯澈,赵忆文,张弼,李英立,赵新刚.基于最优激励位姿序列的机械臂负载估计[J].机器人,2020,42(04):503-512.
 */

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <iostream>

#include <thread>

#include <chrono>

#include <Eigen/Dense>
#include <cmath>


int main(int argc, char *argv[])
{
    ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");

    std::vector<double> TCPPose = rtde_receive.getActualTCPPose();
    std::vector<double> TCPForce = rtde_receive.getActualTCPForce();

    Eigen::Vector3d Torque;
    Torque[0] = -TCPForce[3], Torque[1] = -TCPForce[4], Torque[2] = -TCPForce[5];
    Eigen::Matrix3d P;
    P << 0, -TCPForce[2], TCPForce[1],
        TCPForce[2], 0, -TCPForce[0],
        -TCPForce[1], TCPForce[0], 0;
    Eigen::Vector3d Centroid;
    Centroid = P.inverse() * Torque;

    return 0;
}