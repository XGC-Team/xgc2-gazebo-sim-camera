#include <xgc2_gazebo_scene/ros_master_registration.hpp>
#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <poll.h>
#include <ros/ros.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stopping = 0;
void Stop(int) { stopping = 1; }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_robot_master_fixture", ros::init_options::NoSigintHandler |
            ros::init_options::NoRosout | ros::init_options::NoSimTime);
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  std::atomic<unsigned> first{0}, second{0}, cancelled{0};
  std::atomic<unsigned> desired{1}, applied{0};
  auto first_registration = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([&] {
    applied = desired.load();
    ++first;
  });
  auto second_registration = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([&] { ++second; });
  { xgc2_gazebo_scene::RosMasterRegistration removed([&] { ++cancelled; }); }
  while (!stopping) {
    pollfd input{STDIN_FILENO, POLLIN, 0};
    if (poll(&input, 1, 1000) <= 0) continue;
    std::string command;
    if (!std::getline(std::cin, command) || command == "stop") break;
    if (command.rfind("set ", 0) == 0) desired = std::stoul(command.substr(4));
    const auto status = xgc2_gazebo_scene::CurrentRosMasterStatus();
    std::cout << "{\"state\":\"" << status.state << "\",\"attempts\":" << status.attempts
              << ",\"first\":" << first << ",\"second\":" << second << ",\"cancelled\":" << cancelled
              << ",\"desired_revision\":" << desired << ",\"applied_revision\":" << applied << "}" << std::endl;
  }
  second_registration.reset();
  first_registration.reset();
  ros::shutdown();
}
