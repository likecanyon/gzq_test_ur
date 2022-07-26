/*
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-06-27 20:05:53
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2022-07-24 20:19:48
 * @FilePath: /test/gzq_ur_test/Force_offset.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include <iostream>

#include <thread>

#include <chrono>

using namespace ur_rtde;

int main(int argc, char *argv[])
{
    RTDEControlInterface rtde_control("192.168.3.101");
    RTDEReceiveInterface rtde_receive("192.168.3.101");
    RTDEIOInterface rtde_io("192.168.3.101");
    std::vector<double> Q(6, 0.0);
    Q = rtde_receive.getActualQ();
    std::cout << Q[0] << " " << Q[1] << " " << Q[2]
              << " " << Q[3] << " " << Q[4] << " " << Q[5] << std::endl;
    std::vector<double> TCPForce(6, 0.0);
    std::vector<std::vector<double>> Force_offset(6, std::vector<double>(1000, 0.0));

    std::vector<double> offset_average(6, 0.0);

    TCPForce = rtde_receive.getActualTCPForce();
    std::cout << TCPForce[0] << " " << TCPForce[1] << " " << TCPForce[2]
              << " " << TCPForce[3] << " " << TCPForce[4] << " " << TCPForce[5] << std::endl;
    //利用TCPForce读数，存到Force_offset中
    int i = 0;
    while (i < 1000)

    {
        TCPForce = rtde_receive.getActualTCPForce();

        Force_offset[0].push_back(TCPForce[0]);
        Force_offset[0].erase(Force_offset[0].begin());

        Force_offset[1].push_back(TCPForce[1]);
        Force_offset[1].erase(Force_offset[1].begin());

        Force_offset[2].push_back(TCPForce[2]);
        Force_offset[2].erase(Force_offset[2].begin());

        Force_offset[3].push_back(TCPForce[3]);
        Force_offset[3].erase(Force_offset[3].begin());

        Force_offset[4].push_back(TCPForce[4]);
        Force_offset[4].erase(Force_offset[4].begin());

        Force_offset[5].push_back(TCPForce[5]);
        Force_offset[5].erase(Force_offset[5].begin());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        i++;
    }
    //将1000组数据打印出来
    for (int a = 0; a < 1000; a++)
    {
        for (int b = 0; b < 6; b++)
        {
            std::cout << Force_offset[b][a] << " ";
        }
        std::cout << std::endl;
    }
    //对1000组数据求平均值
    for (int j = 0; j < 6; j++)
    {
        for (int k = 0; k < 1000; k++)
        {
            offset_average[j] = offset_average[j] + Force_offset[j][k];
        }
        offset_average[j] = offset_average[j] / 1000;
    }
    //打印平均值

    for (int l = 0; l < 6; l++)
    {
        std::cout << offset_average[l] << " ";
    }
    std::cout << std::endl;

    return 0;
}