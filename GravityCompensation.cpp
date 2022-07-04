/*
 *tool gravity comepensation in real time
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

//通过x y z Rx Ry Rz获得齐次变换矩阵T
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

//通过x y z Rx Ry Rz 获得旋转变换矩阵R
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

/*
将重力投射到传感器坐标系
 param1 G 重力在m系下的表达；
 R_m_t m系相对于t系的位姿；
 R_s_t
 P 机器人质心位置在s系下的表示。
*/
std::vector<double> ForceTrans(std::vector<double> G, Eigen::Matrix3d R_m_t, std::vector<double> P)
{
    std::vector<double> F(6, 0.0); //重力在传感器坐标系的表达

    Eigen::Vector3d Ft;
    Eigen::VectorXd F_before;
    Eigen::VectorXd F_After;
    Eigen::Vector3d G_w = {G[0], G[1], G[2]};
    Eigen::MatrixXd TransForce;
    Eigen::Matrix3d R_s_t;
    Eigen::Matrix3d SP;
    SP << 0, -P[2], P[1],
        P[2], 0, -P[0],
        -P[1], P[0], 0;

    R_s_t.setIdentity();
    TransForce.setIdentity();

    Ft = R_m_t * G_w; //重力在t系中的表达
    F_before[0] = Ft[0], F_before[1] = Ft[1], F_before[2] = Ft[2];
    F_before[3] = 0, F_before[4] = 0, F_before[5] = 0;
    TransForce.block(0, 0, 3, 3) = R_s_t;
    TransForce.block(3, 3, 3, 3) = R_s_t;
    TransForce.block(3, 0, 3, 3) = -SP * R_s_t;
    F_After = TransForce.inverse() * F_before;

    for (int i = 0; i < 6; i++)
    {
        F[i] = F_After[i];
    }

    return F;
}

int main(int argc, char *argv[])
{
    ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
    ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");

    /******************初始化变量****************************/
    //重力
    double m = 1.0;
    double g = 9.8;
    std::vector<double> G{0.0, 0.0, -m * g}; //重力在m系下的表示
    std::vector<double> FGS(6, 0.0);
    //质心位置
    std::vector<double> Position;

    //传感器读数
    std::vector<double> FS_Source(6, 0.0);

    //补偿后的力
    std::vector<double> FS_AfterCompensation(6, 0.0);

    // m是与原点在质心处，姿态与基座标系平行的参考系;t是工具系，原点在质心处，姿态与传感器坐标系平行；s是传感器坐标系;w是机器人基坐标系
    // R_m_t t是参考系
    Eigen::Matrix3d R_m_t;
    Eigen::Matrix3d R_m_w;

    // R_t_s s是参考系
    Eigen::Matrix3d R_t_s;
    Eigen::Matrix4d T_t_s;

    // R_s_w 是参考系
    Eigen::Matrix3d R_s_w;
    // T_s_w w是参考系
    Eigen::Matrix4d T_s_w;

    // TCPPose
    std::vector<double> TCPPoseVector(6, 0.0);
    Eigen::Vector3d K;
    double alpha;
    Eigen::Matrix4d T_end_base; // base是参考系，机器人基坐标系

    /*************重力补偿计算*************************/
    //读取机器人传感器坐标系相对于基坐标系的位姿
    TCPPoseVector = rtde_receive.getActualTCPPose();
    T_s_w = GetHomoTransMatrix(TCPPoseVector);
    R_s_w = GetRotationMatrix(TCPPoseVector);

    //计算各个坐标系的相对姿态关系
    R_m_w.setIdentity();     // m坐标系与机器人基坐标系姿态平行；
    R_t_s.setIdentity();     // t坐标系与机器人传感器坐标系平行；
    R_m_t = R_s_w.inverse(); // R_m_t=R_w_s
    FS_Source = rtde_receive.getActualTCPForce();
    FGS = ForceTrans(G, R_m_t, Position);

    //减去重力引起的传感器变化
    for (int i = 0; i < 6; i++)
    {
        FS_AfterCompensation[i] = FS_Source[i] - FGS[i];
    }
    /************************计算完成***************************/

    return 0;
}