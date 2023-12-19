#ifndef ESKF_LIO_IMU_SUBSCRIBER_HPP_
#define ESKF_LIO_IMU_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ESKF_LIO/Types.hpp"
#include "ESKF_LIO/SynchronizedQueue.hpp"

class ImuSubscriber : public rclcpp::Node
{
public:
  using MsgType = typename sensor_msgs::msg::Imu;
  using Buffer = SynchronizedQueue<ESKF_LIO::ImuMeasurementPtr>;
  using BufferPtr = std::shared_ptr<Buffer>;

  ImuSubscriber(const std::string & topicName)
  : Node("ImuSubscriber"), buffer_(std::make_shared<Buffer>())
  {
    rclcpp::QoS qosProfile(10);
    subscriber_ = this->create_subscription<MsgType>(
      topicName, qosProfile,
      std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1));
  }
  virtual ~ImuSubscriber() {}

  BufferPtr getBuffer() const
  {
    return buffer_;
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg) const
  {
    auto measurement = std::make_shared<ESKF_LIO::ImuMeasurement>();
    measurement->timestamp = rclcpp::Time(imuMsg->header.stamp).seconds();
    const auto & angular_velocity = imuMsg->angular_velocity;
    const auto & acceleration = imuMsg->linear_acceleration;
    measurement->angularVelocity << angular_velocity.x,
      angular_velocity.y,
      angular_velocity.z;
    measurement->acceleration << acceleration.x,
      acceleration.y,
      acceleration.z;

    buffer_->push(std::move(measurement));
  }

  rclcpp::Subscription<MsgType>::SharedPtr subscriber_;
  BufferPtr buffer_;
};

class LidarSubscriber : public rclcpp::Node
{
public:
  using MsgType = typename sensor_msgs::msg::PointCloud2;
  using Buffer = SynchronizedQueue<ESKF_LIO::LidarMeasurementPtr>;
  using BufferPtr = std::shared_ptr<Buffer>;

  LidarSubscriber(const std::string & topicName)
  : Node("LidarSubscriber"), buffer_(std::make_shared<Buffer>())
  {
    rclcpp::QoS qosProfile(10);
    subscriber_ = this->create_subscription<MsgType>(
      topicName, qosProfile,
      std::bind(&LidarSubscriber::cloudCallback, this, std::placeholders::_1));
  }

  BufferPtr getBuffer() const
  {
    return buffer_;
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) const
  {
    auto measurement = std::make_shared<ESKF_LIO::LidarMeasurement>();

    const size_t numPoints = cloudMsg->height * cloudMsg->width;
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_.resize(numPoints);
    measurement->pointTime.resize(numPoints);

    sensor_msgs::PointCloud2ConstIterator<float> x_iter(*cloudMsg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(*cloudMsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(*cloudMsg, "z");
    sensor_msgs::PointCloud2ConstIterator<double> t_iter(*cloudMsg, "timestamp");

    for (size_t i = 0; i < numPoints; ++i, ++x_iter, ++y_iter, ++z_iter, ++t_iter) {
      cloud->points_[i] << *x_iter, *y_iter, *z_iter;
      measurement->pointTime[i] = *t_iter;
    }
    measurement->startTime = measurement->pointTime.front();
    measurement->endTime = measurement->pointTime.back();
    measurement->cloud = std::move(cloud);

    buffer_->push(measurement);
  }

  rclcpp::Subscription<MsgType>::SharedPtr subscriber_;
  BufferPtr buffer_;
};

#endif // ESKF_LIO_IMU_SUBSCRIBER_HPP_
