#include "stereo_ugv/image_undistorter.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/cudawarping.hpp>

namespace stereo_ugv
{
/**
 * @brief Sets the image source.
 * @param source The image source.
 */
void ImageUndistorter::setImageSource(std::unique_ptr<ImageSource>&& source)
{
  image_source_ = std::move(source);
}

/**
 * @brief Calculates maps for undistortion and stores them in internal variables.
 * @warning This function should only be called after setting the image source because this function needs to know the
 * size of input images.
 * @param coefficients Undistortion coefficients calculated by the LensCalibrator.
 * @param new_image_size The size of the undistorted images. If the input argument is 0x0, the same size as the input
 * images will be used instead.
 */
void ImageUndistorter::calculateUndistortionMaps(const LensCoefficients& coefficients, cv::Size new_image_size)
{
  STEREO_UGV_CHECK_PARAMETER((new_image_size.width > 0 && new_image_size.height > 0) ||
                             (new_image_size.width == 0 && new_image_size.height == 0));

  cv::Mat new_camera[2];
  cv::Mat rectification[2];
  cv::stereoRectify(coefficients.camera_matrices[0], coefficients.distortion_coefficients[0],
                    coefficients.camera_matrices[1], coefficients.distortion_coefficients[1],
                    image_source_->imageSize(), coefficients.rotation_matrix, coefficients.translation_vector,
                    rectification[0], rectification[1], new_camera[0], new_camera[1], Q_, cv::CALIB_ZERO_DISPARITY, 1,
                    new_image_size);

  for (auto i{ 0 }; i < 2; ++i)
  {
    cv::Mat map_x, map_y;
    cv::initUndistortRectifyMap(coefficients.camera_matrices[i], coefficients.distortion_coefficients[i],
                                rectification[i], new_camera[i], new_image_size, CV_32FC1, map_x, map_y);
    map_x_[i].upload(map_x);
    map_y_[i].upload(map_y);
  }
}

/**
 * @brief Reads a pair of images from the internal image source and undistorts them.
 * @param left The undistorted image taken by the left head.
 * @param right The undistorted image taken by the right head.
 */
void ImageUndistorter::undistort(cv::cuda::GpuMat* left, cv::cuda::GpuMat* right) const
{
  image_source_->readImages(&images_[0], &images_[1]);
  for (auto i{ 0 }; i < 2; ++i)
  {
    cuda_images_[i].upload(images_[i]);
  }
  cv::cuda::remap(cuda_images_[0], *left, map_x_[0], map_y_[0], cv::INTER_LINEAR);
  cv::cuda::remap(cuda_images_[1], *right, map_x_[1], map_y_[1], cv::INTER_LINEAR);
}

/**
 * @brief Initializes an ImageUndistorter.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   "imageSource": <ImageSource>,
 *   "undistortionFile": <string>,
 *   "newImageSize": <cv::Size>
 * }
 * @endcode
 * @param undistorter The image undistorter to be initialized.
 * @param context The context.
 */
void initialize(ImageUndistorter* undistorter, const Context& context)
{
  undistorter->setImageSource(ImageSource::create({ context, "imageSource" }));

  std::string undistortion_file;
  context.getParameter("undistortionFile", &undistortion_file);
  cv::FileStorage storage;
  if (!storage.open(undistortion_file, cv::FileStorage::READ))
  {
    throw RuntimeError{ fmt::format(R"(Could not read from undistortion file "{}")", undistortion_file) };
  }

  LensCoefficients coefficients;

  storage["camera_matrix_left"] >> coefficients.camera_matrices[0];
  storage["camera_matrix_right"] >> coefficients.camera_matrices[1];

  storage["distortion_coefficients_left"] >> coefficients.distortion_coefficients[0];
  storage["distortion_coefficients_right"] >> coefficients.distortion_coefficients[1];

  storage["rotation_matrix"] >> coefficients.rotation_matrix;
  storage["translation_vector"] >> coefficients.translation_vector;

  cv::Size new_image_size;
  context.getParameter("newImageSize", &new_image_size);
  undistorter->calculateUndistortionMaps(coefficients, new_image_size);
}

}  // namespace stereo_ugv
