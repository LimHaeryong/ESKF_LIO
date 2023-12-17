#include <iostream>
#include <thread>

#include "ESKF_LIO/Odometry.hpp"
#include "ESKF_LIO/Subscriber.hpp"
#include "ESKF_LIO/Types.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto imuSubscriber = std::make_shared<ImuSubscriber>("/alphasense/imu");
    auto lidarSubscriber = std::make_shared<LidarSubscriber>("/hesai/pandar");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(imuSubscriber);
    executor->add_node(lidarSubscriber);

    auto imuBuffer = imuSubscriber->getBuffer();
    auto cloudBuffer = lidarSubscriber->getBuffer();

    std::thread executorThread([&executor]()
                               { executor->spin(); });

    while (true)
    {
        auto imuMeas = imuBuffer->popAll();
        auto lidarMeas = cloudBuffer->pop();

        if(!imuMeas.empty())
        {
            std::cout << "imu vector size : " << imuMeas.size() << "\n";
        }
        for (auto &imu : imuMeas)
        {
            std::cout << "imu subscribed : " << std::fixed << std::setprecision(9) << imu->timestamp << "\n";
        }

        if (lidarMeas.has_value())
        {
            std::cout << "lidar subscribed : " << std::fixed << std::setprecision(9) << lidarMeas.value()->timestamp << "\n";
            std::cout << "lidar first point time : " << std::fixed << std::setprecision(9) << lidarMeas.value()->pointTime.front() << "\n";
            std::cout << "lidar last point time : " << std::fixed << std::setprecision(9) << lidarMeas.value()->pointTime.back() << "\n";
        }
    }

    executor->cancel();
    executorThread.join();
    rclcpp::shutdown();

    return 0;
}