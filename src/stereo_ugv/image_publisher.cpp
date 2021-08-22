#include "stereo_ugv/image_publisher.h"

#include <ros/assert.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>

namespace stereo_ugv
{
/**
 * @brief Converts a cv::Mat of BGR8 color type to sensor_msgs::Image.
 * @param image The destination image message.
 * @param matrix The source matrix.
 */
void CvMatConverter::operator()(sensor_msgs::Image* image, const cv::Mat& matrix) const
{
  ROS_ASSERT(!matrix.empty() && matrix.type() == CV_8UC3);
  image->encoding = sensor_msgs::image_encodings::BGR8;

  const auto size{ matrix.size() };
  image->width = static_cast<uint32_t>(size.width);
  image->height = static_cast<uint32_t>(size.height);
  image->step = 3 * image->width;

  image->data.resize(image->height * image->step);
  auto destination{ image->data.data() };
  for (auto i{ 0 }; i < size.height; ++i)
  {
    std::copy_n(matrix.ptr(i), image->step, destination);
    destination += image->step;
  }

  image->is_bigendian = 0;
}

/**
 * @brief Converts a cv::cuda::GpuMat to sensor_msgs::Image.
 * @param image The destination image message.
 * @param matrix The source matrix.
 */
void CvGpuMatConverter::operator()(sensor_msgs::Image* image, const cv::cuda::GpuMat& matrix) const
{
  matrix.download(matrix_);
  CvMatConverter::operator()(image, matrix_);
}

}  // namespace stereo_ugv
