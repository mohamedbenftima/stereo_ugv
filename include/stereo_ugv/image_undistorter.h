/**
 * @file image_undistorter.h
 * @brief Defines stereo_ugv::ImageUndistorter.
 */

#ifndef STEREO_UGV_IMAGE_UNDISTORTER_H
#define STEREO_UGV_IMAGE_UNDISTORTER_H

#include "stereo_ugv/lens_calibrator.h"

#include <opencv2/core/cuda.hpp>

namespace stereo_ugv
{
/**
 * @brief The class for undistorting images taken by a stereo camera such that they are ready for disparity estimation.
 */
class ImageUndistorter
{
public:
  void setImageSource(std::unique_ptr<ImageSource>&& source);

  void calculateUndistortionMaps(const LensCoefficients& coefficients, cv::Size new_image_size);

  void undistort(cv::cuda::GpuMat* left, cv::cuda::GpuMat* right) const;

private:
  std::unique_ptr<ImageSource> image_source_;
  cv::Mat Q_;
  cv::cuda::GpuMat map_x_[2];
  cv::cuda::GpuMat map_y_[2];
  mutable cv::Mat images_[2];
  mutable cv::cuda::GpuMat cuda_images_[2];
};

void initialize(ImageUndistorter* undistorter, const Context& context);

}  // namespace stereo_ugv

#endif  // STEREO_UGV_IMAGE_UNDISTORTER_H
