cmake_minimum_required(VERSION 3.5)
project(gzq_test_ur)

find_package(ur_rtde REQUIRED)
add_subdirectory(3rdparty/ruckig)
include_directories(3rdparty/ruckig/include)

add_executable(FTsensors FTsensors.cpp)
target_link_libraries(FTsensors PUBLIC ur_rtde::rtde)

add_executable(Force_offset Force_offset.cpp)
target_link_libraries(Force_offset PUBLIC ur_rtde::rtde)

add_executable(GetSomeData GetSomeData.cpp Gravity.cpp)
target_link_libraries(GetSomeData PUBLIC ur_rtde::rtde)

add_executable(GravityCompensation GravityCompensation.cpp)
target_link_libraries(GravityCompensation PUBLIC ur_rtde::rtde)

add_executable(CentroidofMass CentroidofMass.cpp)
target_link_libraries(CentroidofMass PUBLIC ur_rtde::rtde)

add_executable(ForcePlan ForcePlan.cpp)
target_link_libraries(ForcePlan PUBLIC ur_rtde::rtde  ruckig::ruckig)