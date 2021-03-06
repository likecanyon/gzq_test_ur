/*
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-06-28 20:47:13
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-07-25 22:20:16
 * @FilePath: /gzq_ur_test/GetSomeData.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-06-28 20:47:13
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-07-19 22:26:00
 * @FilePath: /test/gzq_ur_test/GetSomeData.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <iostream>

#include <thread>

#include <chrono>

#include <Eigen/Dense>
#include <cmath>

using namespace ur_rtde;
#include "Gravity.h"
Eigen::Matrix4d GetHomoTransMatrix(std::vector<double> I);
Eigen::Matrix3d GetRotationMatrix(std::vector<double> I);
std::vector<double> ForceTrans(std::vector<double> G, Eigen::Matrix3d R_m_t, std::vector<double> P);
std::vector<double> FromWtoS(std::vector<double> F, Eigen::Matrix3d R_s_w);
std::vector<double> FromStoW(std::vector<double> F, Eigen::Matrix3d R_s_w);
std::vector<double> ForceTrans(std::vector<double> G, Eigen::Matrix3d R_m_t, std::vector<double> P);
int main(int argc, char *argv[])
{
  RTDEControlInterface rtde_control("192.168.3.101");
  RTDEReceiveInterface rtde_receive("192.168.3.101");
  Eigen::Matrix3d R_s_t;
  Eigen::Matrix3d R_s_w;
  std::vector<double> InitQ{0, -1.57, -1.57, -1.57, 1.57, 0};
  // home:0,-90,0,-90,0,00.0, -1.57, -1.57, -1.57, 1.57, 0

  rtde_control.moveJ(InitQ);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  rtde_control.zeroFtSensor();
  std::vector<double> TCPForce(6, 0.0);
  std::vector<double> TCPPose(6, 0.0);
  TCPPose = rtde_receive.getActualTCPPose();
  R_s_w = GetRotationMatrix(TCPPose);
  GravityCompensation();
  TCPForce = rtde_receive.getActualTCPForce();
  // TCPForce = FromStoW(TCPForce, R_s_w);
  std::cout << "the TCPForce is: " << TCPForce[0] << " " << TCPForce[1] << " " << TCPForce[2] << " " << TCPForce[3]
            << " " << TCPForce[4] << " " << TCPForce[5] << std::endl;

  InitQ[3] = 1.57;
  rtde_control.moveJ(InitQ);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  TCPForce = rtde_receive.getActualTCPForce();
  std::cout << "the TCPForce is: " << TCPForce[0] << " " << TCPForce[1] << " " << TCPForce[2] << " " << TCPForce[3]
            << " " << TCPForce[4] << " " << TCPForce[5] << std::endl;

  return 0;
}