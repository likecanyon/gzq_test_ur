#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("192.168.3.101");
  RTDEReceiveInterface rtde_receive("192.168.3.101");

  std::vector<double> TCPForce(6, 0.0);
  TCPForce = rtde_receive.getActualTCPForce();
  std::cout << "the TCPForce is: " << TCPForce[0] << " " << TCPForce[1] << " " << TCPForce[2] << " " << TCPForce[3]
            << " " << TCPForce[4] << " " << TCPForce[5] << std::endl;
  rtde_control.zeroFtSensor();
  std::cout << "the TCPForce is: " << TCPForce[0] << " " << TCPForce[1] << " " << TCPForce[2] << " " << TCPForce[3]
            << " " << TCPForce[4] << " " << TCPForce[5] << std::endl;
  return 0;
}