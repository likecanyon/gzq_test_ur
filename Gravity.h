/*
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-07-19 16:17:32
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-07-19 22:26:39
 * @FilePath: /gzq_ur_test/GravityCompensation.h
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
Eigen::Matrix4d GetHomoTransMatrix(std::vector<double> I);
Eigen::Matrix3d GetRotationMatrix(std::vector<double> I);
std::vector<double> ForceTrans(std::vector<double> G, Eigen::Matrix3d R_m_t, std::vector<double> P);
std::vector<double> FromWtoS(std::vector<double> F, Eigen::Matrix3d R_s_w);
std::vector<double> FromStoW(std::vector<double> F, Eigen::Matrix3d R_s_w);
std::vector<double> ForceTrans(std::vector<double> G, Eigen::Matrix3d R_m_t, std::vector<double> P);
std::vector<double> GravityCompensation();