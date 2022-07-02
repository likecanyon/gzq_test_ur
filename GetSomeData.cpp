#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

int main(int argc, char *argv[])
{RTDEControlInterface rtde_control("192.168.3.101");
  RTDEReceiveInterface rtde_receive("192.168.3.101");

  std::vector<double> Q(6, 0.0);
  Q=rtde_receive.getActualQ();
  std::cout<<Q[0]<<" "<<Q[1]<<" "<<Q[2]<<" "<<Q[3]<<" "<<Q[4]<<" "<<Q[5]<<std::endl;

  return 0;
}