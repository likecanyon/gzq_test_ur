
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <iostream>

#include <thread>

#include <chrono>

#include <Eigen/Dense>
#include <cmath>

using namespace ur_rtde;

int main(int argc, char *argv[])
{
  RTDEControlInterface rtde_control("192.168.3.101");
  RTDEReceiveInterface rtde_receive("192.168.3.101");
  Eigen::Matrix3d R_s_t;

  std::vector<double> InitQ{0, -1.57, -1.57, -1.57, 1.57, 0}; // home:0,-90,0,-90,0,00.0, -1.57, -1.57, -1.57, 1.57, 0
  rtde_control.moveJ(InitQ);
  std::vector<double> TCPForce(6, 0.0);
  TCPForce = rtde_receive.getActualTCPForce();
  std::cout << "the TCPForce is: " << TCPForce[0] << " " << TCPForce[1] << " " << TCPForce[2] << " " << TCPForce[3]
            << " " << TCPForce[4] << " " << TCPForce[5] << std::endl;

  return 0;
}