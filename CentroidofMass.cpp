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

Eigen::Matrix3d GetRotationMatrix(std::vector<double> I)
{
    Eigen::Vector3d K = {I[3], I[4], I[5]};
    double alpha;
    Eigen::Matrix3d R;
    K.normalize();
    alpha = I[3] / K[0];
    R = Eigen::AngleAxisd(alpha, K);

    return R;
}

//将基坐标系的力转到传感器坐标系表示
std::vector<double> FromWtoS(std::vector<double> F, Eigen::Matrix3d R_s_w)
{
    std::vector<double> F_S; //传感器坐标系下的广义力

    Eigen::Vector3d ForceW; //基坐标系表示的力
    Eigen::Vector3d MW;     //基坐标系表示的力矩
    ForceW[0] = F[0], ForceW[1] = F[1], ForceW[2] = F[2];
    MW[0] = F[3], MW[1] = F[4], MW[2] = F[5];
    ForceW = R_s_w.inverse() * ForceW;
    MW = R_s_w * MW;
    F_S[0] = ForceW[0], F_S[1] = ForceW[1], F_S[2] = ForceW[2];
    F_S[3] = MW[0], F_S[4] = MW[1], F_S[5] = MW[2];

    return F_S;
}

//将传感器坐标系的力转换到基坐标系表示
std::vector<double> FromStoW(std::vector<double> F, Eigen::Matrix3d R_s_w)
{
    std::vector<double> F_W; //基坐标系的广义力
    Eigen::Vector3d ForceS;  //传感器坐标系的力
    Eigen::Vector3d MS;      //传感器坐标系的力矩
    ForceS[0] = F[0], ForceS[1] = F[1], ForceS[2] = F[2];
    MS[0] = F[3], MS[1] = F[4], MS[2] = F[5];
    ForceS = R_s_w * ForceS;
    MS = R_s_w * MS;
    F_W[0] = ForceS[0], F_W[1] = ForceS[1], F_W[2] = ForceS[2];
    F_W[3] = MS[0], F_W[4] = MS[1], F_W[5] = MS[2];
    return F_W;
}

int main(int argc, char *argv[])
{
    ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");

    std::vector<double> TCPPose = rtde_receive.getActualTCPPose();
    Eigen::Matrix3d R_s_w;
    R_s_w = GetRotationMatrix(TCPPose);

    std::vector<double> TCPForce = rtde_receive.getActualTCPForce();

    TCPForce = FromStoW(TCPForce, R_s_w);
    Eigen::Vector3d Torque;

    Torque[0] = -TCPForce[3], Torque[1] = -TCPForce[4], Torque[2] = -TCPForce[5];
    Eigen::Matrix3d P;
    P << 0, -TCPForce[2], TCPForce[1],
        TCPForce[2], 0, -TCPForce[0],
        -TCPForce[1], TCPForce[0], 0;
    Eigen::Vector3d Centroid;
    Centroid = P.inverse() * Torque;

    std::cout << Centroid[0] << " " << Centroid[1] << " " << Centroid[2] << std::endl;

    return 0;
}