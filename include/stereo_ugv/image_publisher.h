/**
 * @file image_publisher.h
 * @brief Defines stereo_ugv::ImagePublisher<T> template class and common image converters.
 */

#ifndef STEREO_UGV_IMAGE_PUBLISHER_H
#define STEREO_UGV_IMAGE_PUBLISHER_H

#include <opencv2/core/cuda.hpp>
#include <opencv2/core/mat.hpp>

#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>

#include <optional>
#include <utility>

namespace stereo_ugv
{
/**
 * @brief A utility class for easily setting up an image publisher from context parameters and publishing images through
 * this publisher.
 * @tparam T The base class that provides the convertion method from data of custom types and numbers to image messages.
 * @details The template argument T should be default-constructible should defines operator():
 * @code
 * void operator(sensor_msgs::Image* image, const T1 &argument1, const T2 &argument 2, ...) const;
 * @endcode
 * where argument1, argument2, ... are arguments used to create image.
 */
template <typename T>
class ImagePublisher : public T
{
public:
  using T::T;

  /**
   * @brief Enables publishing images by setting the internal image_transport::Publisher.
   * @param publisher The internal image publisher.
   */
  void setPublisher(image_transport::Publisher&& publisher)
  {
    publisher_ = std::move(publisher);
  }

  /**
   * @brief Disables publishing images by unsetting the internal image_transport::Publisher.
   */
  void unsetPublisher()
  {
    publisher_.reset();
  }

  /**
   * @brief Publishes an image through the internal image publisher if it is enabled. Otherwise, does nothing.
   * @param arguments Arguments that are used to create the image message to be published. These arguments, together
   * with a pointer to the internal image buffer, are passed to the T\::operator() function to create the image message,
   * where T is the template argument to this class.
   */
  template <typename... Arguments>
  void publish(Arguments&&... arguments) const
  {
    if (publisher_.has_value())
    {
      T::operator()(&image_, std::forward(arguments)...);
      publisher_->publish(image_);
    }
  }

private:
  std::optional<image_transport::Publisher> publisher_;
  mutable sensor_msgs::Image image_;
};

/**
 * @brief The converter class from cv::Mat to sensor_msgs::Image.
 * @warning Currently, only matrices of BGR8 color type are supported. Passing matrices of other types causes undefined
 * behaviour.
 */
class CvMatConverter
{
public:
  void operator()(sensor_msgs::Image* image, const cv::Mat& matrix) const;
};

/**
 * @brief The converter class from cv::cuda::GpuMat to sensor_msgs::Image.
 * @warning Currently, only matrices of BGR8 color type are supported. Passing matrices of other types causes undefined
 * behaviour.
 */
class CvGpuMatConverter : protected CvMatConverter
{
public:
  void operator()(sensor_msgs::Image* image, const cv::cuda::GpuMat& matrix) const;

private:
  cv::Mat matrix_;
};

}  // namespace stereo_ugv

#endif  // STEREO_UGV_IMAGE_PUBLISHER_H
