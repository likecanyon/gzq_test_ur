#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <iostream>

#include <thread>

#include <chrono>

#include <Eigen/Dense>
#include <cmath>

Eigen::Matrix4d GetHomoTransMatrix(std::vector<double> I)
{
    Eigen::Vector3d K = {I[3], I[4], I[5]};
    Eigen::Vector3d Translation = {I[0], I[1], I[2]};
    double alpha;
    Eigen::Matrix4d T;
    T.setIdentity();
    Eigen::Matrix3d R;
    K.normalize();
    alpha = I[3] / K[0];
    R = Eigen::AngleAxisd(alpha, K);
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = Translation;

    return T;
}

Eigen::Matrix4d GetRotationMatrix(std::vector<double> I)
{
    Eigen::Vector3d K = {I[3], I[4], I[5]};
    double alpha;
    Eigen::Matrix3d R;
    K.normalize();
    alpha = I[3] / K[0];
    R = Eigen::AngleAxisd(alpha, K);

    return R;
}

std::vector<double> ForceTrans(std::vector<double> A, Eigen::Matrix4d B)
{
    std::vector<double> F(6, 0.0);

    return F;
}

int main(int argc, char *argv[])
{
    ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");

    //重力
    std::vector<double> G{0.0, 0.0, 0.0};

    //传感器读数
    std::vector<double> FS_Source(6, 0.0);

    //补偿后的力
    std::vector<double> FS_AfterCompensation(6, 0.0);

    // m是与原点在质心处，姿态与基座标系平行的参考系;t是工具系，原点在质心处，姿态与传感器坐标系平行；s是传感器坐标系。
    // R_m_t t是参考系
    Eigen::Matrix3d R_m_t;

    // R_t_s s是参考系
    Eigen::Matrix3d R_t_s;

    // TCPPose
    std::vector<double> TCPPoseVector(6, 0.0);
    Eigen::Vector3d K;
    double alpha;
    Eigen::Matrix4d T_end_base; // base是参考系，机器人基坐标系

    TCPPoseVector = rtde_receive.getActualTCPPose();

    return 0;
}