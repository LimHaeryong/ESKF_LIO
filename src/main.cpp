#include <iostream>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "ESKF_LIO/Odometry.hpp"
#include "ESKF_LIO/Subscriber.hpp"
#include "ESKF_LIO/Types.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto config = YAML::LoadFile(CONFIG_PATH);
    auto imuTopic = config["sensors"]["imu"]["topic_name"].as<std::string>();
    auto lidarTopic = config["sensors"]["lidar"]["topic_name"].as<std::string>();

    auto imuSubscriber = std::make_shared<ImuSubscriber>(imuTopic);
    auto lidarSubscriber = std::make_shared<LidarSubscriber>(lidarTopic);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(imuSubscriber);
    executor->add_node(lidarSubscriber);

    auto imuBuffer = imuSubscriber->getBuffer();
    auto cloudBuffer = lidarSubscriber->getBuffer();

    auto odom = std::make_shared<ESKF_LIO::Odometry>(config, imuBuffer, cloudBuffer);

    std::thread executorThread([&executor]()
                               { executor->spin(); });

    odom->run();

    executor->cancel();
    executorThread.join();
    rclcpp::shutdown();

    return 0;
}