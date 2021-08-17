#include "stereo_ugv/lens_calibrator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

#include <ros/assert.h>

#include <cmath>

namespace stereo_ugv
{
/**
 * @brief Gets the undistortion coefficients calculated by calculateCoefficients().
 * @return The undistortion coefficients.
 */
const LensCoefficients& LensCalibrator::coefficients() const noexcept
{
  return coefficients_;
}

/**
 * @brief Sets the image source.
 * @param source The image source.
 */
void LensCalibrator::setImageSource(std::unique_ptr<ImageSource>&& source)
{
  image_source_ = std::move(source);
}

/**
 * @brief Sets the maximum number of image pairs used for calibration.
 * @param count The maximum number of frames in which images taken by both heads have detected feature points. The
 * calibrator will stop reading more images after reaching this limit. A zero value means there is no such limit.
 */
void LensCalibrator::setMaxImageCount(std::size_t count)
{
  max_image_count_ = count;
}

/**
 * @brief The filename to which the calculated undistortion coefficients are written.
 * @param filename The name of the undistortion file.
 */
void LensCalibrator::setUndistortionFilename(std::string filename)
{
  undistortion_filename_ = std::move(filename);
}

/**
 * @brief Calculates the undistortion coefficients of the stereo camera and stores them in the internal
 * LensCoefficients.
 */
void LensCalibrator::calculateCoefficients()
{
  std::vector<std::vector<cv::Point2f>> image_points[2];
  std::vector<std::vector<cv::Point2f>> common_image_points[2];

  try
  {
    if (max_image_count_ == 0)
    {
      while (true)
      {
        detectFramePoints(image_points, common_image_points);
      }
    }
    else
    {
      do
      {
        detectFramePoints(image_points, common_image_points);
      } while (common_image_points[0].size() < max_image_count_);
    }
  }
  catch (const OutOfFrames& exception)
  {
    ROS_INFO_STREAM(exception.what());
  }

  ROS_ASSERT(common_image_points[0].size() == common_image_points[1].size());
  if (common_image_points[0].empty())
  {
    throw RuntimeError{
      "Could not calibrate the camera because there is no frame where both heads detect the calibration pattern"
    };
  }
  ROS_INFO_STREAM("Detected " << image_points[0].size() << " images for calibrating the left head");
  ROS_INFO_STREAM("Detected " << image_points[1].size() << " images for calibration the right head");
  ROS_INFO_STREAM("Detected " << common_image_points[0].size() << " frames for stereo calibration");

  std::vector<std::vector<cv::Point3f>> object_points{ createObjectPoints() };
  const auto image_size{ image_source_->imageSize() };
  const auto i_fixed_point{ indexOfFixedPoint() };
  cv::Mat r_vectors, t_vectors, new_object_points, essential_matrix, fundamental_matrix;

  for (auto i{ 0 }; i < 2; ++i)
  {
    object_points.resize(image_points[i].size(), object_points.front());
    coefficients_.camera_matrices[i] = cv::Mat{ cv::Size{ 3, 3 }, CV_64F };
    coefficients_.distortion_coefficients[i] = cv::Mat{ cv::Size{ 5, 1 }, CV_64F };
    const auto error{ cv::calibrateCameraRO(object_points, image_points[i], image_size, i_fixed_point,
                                            coefficients_.camera_matrices[i], coefficients_.distortion_coefficients[i],
                                            r_vectors, t_vectors, new_object_points) };
    ROS_INFO_STREAM("Calibration error for the " << (i ? "right" : "left") << " head is " << error);
  }

  object_points.resize(common_image_points[0].size(), object_points.front());
  const auto error{ cv::stereoCalibrate(object_points, common_image_points[0], common_image_points[1],
                                        coefficients_.camera_matrices[0], coefficients_.distortion_coefficients[0],
                                        coefficients_.camera_matrices[1], coefficients_.distortion_coefficients[1],
                                        image_size, coefficients_.rotation_matrix, coefficients_.translation_vector,
                                        essential_matrix, fundamental_matrix) };
  ROS_INFO_STREAM("Stereo calibration error is " << error);
}

/**
 * @brief Saves the undistortion coefficients calculated by calculateCoefficients() into a YAML or XML file whose name
 * is specified by setUndistortionFilename().
 */
void LensCalibrator::saveCoefficients()
{
  cv::FileStorage storage;
  if (!storage.open(undistortion_filename_, cv::FileStorage::WRITE))
  {
    throw RuntimeError{ fmt::format(R"(Could not write to undistortion file "{}")", undistortion_filename_) };
  }

  storage << "camera_matrix"
          << "left" << coefficients_.camera_matrices[0];
  storage << "camera_matrix"
          << "right" << coefficients_.camera_matrices[1];

  storage << "distortion_coefficients"
          << "left" << coefficients_.distortion_coefficients[0];
  storage << "distortion_coefficients"
          << "right" << coefficients_.distortion_coefficients[1];

  storage << "rotation_matrix" << coefficients_.rotation_matrix;
  storage << "translation_vector" << coefficients_.translation_vector;
}

void LensCalibrator::detectFramePoints(std::vector<std::vector<cv::Point2f>> image_points[2],
                                       std::vector<std::vector<cv::Point2f>> common_image_points[2]) const
{
  image_source_->readImages(&images_[0], &images_[1]);
  auto both_detected{ true };

  for (auto i{ 0 }; i < 2; ++i)
  {
    std::vector<cv::Point2f> points;
    if (detectImagePoints(images_[i], &points))
    {
      image_points[i].push_back(std::move(points));
    }
    else
    {
      both_detected = false;
    }
  }

  if (both_detected)
  {
    for (auto i{ 0 }; i < 2; ++i)
    {
      common_image_points[i].push_back(image_points[i].back());
    }
  }
}

/**
 * @brief Initializes a LensCalibrator.
 * @warning Because LensCalibrator is an abstract class, this function should be called in the initialization functions
 * of the subclasses.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   "imageSource": <ImageSource>,
 *   "maxImageCount": <integer>,
 *   "undistortionFilename": <string>
 *   "type": <string>
 *   ...
 * }
 * @endcode
 * where ... means type-specific entries.
 * @param calibrator The lens calibrator to be initialized.
 * @param context The context.
 */
void initialize(LensCalibrator* calibrator, const Context& context)
{
  calibrator->setImageSource(ImageSource::create({ context, "imageSource" }));

  std::size_t max_image_count;
  context.getParameter("maxImageCount", &max_image_count, std::size_t{ 0 });
  calibrator->setMaxImageCount(max_image_count);

  std::string undistortion_filename;
  context.getParameter("undistortionFilename", &undistortion_filename);
  calibrator->setUndistortionFilename(std::move(undistortion_filename));
}

/**
 * @brief Sets the size of the chessboard pattern.
 * @param size The pattern size.
 */
void ChessboardLensCalibrator::setPatternSize(cv::Size size)
{
  STEREO_UGV_CHECK_PARAMETER(size.width > 0 && size.height > 0);
  pattern_size_ = size;
}

/**
 * @brief Sets the square length of the chessboard pattern.
 * @param length The square length.
 */
void ChessboardLensCalibrator::setSquareLength(float length)
{
  STEREO_UGV_CHECK_PARAMETER(length > 0 && std::isfinite(length));
  square_length_ = length;
}

/**
 * @brief Sets whether the method of releasing object is enabled.
 * @details If the method of releasing object is enabled, the last point of the first row will be chosen as the
 * third fixed point in cv::calibrateCameraRO().
 * @param enabled Whether the method of releasing object is enabled.
 */
void ChessboardLensCalibrator::setReleaseObject(bool enabled)
{
  release_object_ = enabled;
}

/**
 * @brief Sets whether the chessboard pattern has markers.
 * @param enabled Whether the chessboard pattern has markers.
 */
void ChessboardLensCalibrator::setMarker(bool enabled)
{
  marker_ = enabled;
}

bool ChessboardLensCalibrator::detectImagePoints(const cv::Mat& image, std::vector<cv::Point2f>* points) const
{
  auto flags{ cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY };
  if (marker_)
  {
    flags |= cv::CALIB_CB_MARKER;
  }
  return cv::findChessboardCornersSB(image, pattern_size_, *points, flags);
}

std::vector<cv::Point3f> ChessboardLensCalibrator::createObjectPoints() const
{
  std::vector<cv::Point3f> points;
  for (auto i{ 0 }; i < pattern_size_.height; ++i)
  {
    for (auto j{ 0 }; j < pattern_size_.width; ++j)
    {
      points.emplace_back(j * square_length_, i * square_length_, 0);
    }
  }
  return points;
}

int ChessboardLensCalibrator::indexOfFixedPoint() const noexcept
{
  return release_object_ ? pattern_size_.width - 1 : 0;
}

/**
 * @brief Initializes a ChessboardLensCalibrator.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   ...
 *   patternSize: <cv::Size>,
 *   squareLength: <number>,
 *   releaseObject: <boolean>,
 *   marker: <boolean>
 * }
 * @endcode
 * where ... means entries of the superclass.
 * @param calibrator The lens calibrator to be initialized.
 * @param context The context.
 */
void initialize(ChessboardLensCalibrator* calibrator, const Context& context)
{
  initialize(static_cast<LensCalibrator*>(calibrator), context);

  cv::Size pattern_size;
  context.getParameter("patternSize", &pattern_size);
  calibrator->setPatternSize(pattern_size);

  float square_length;
  context.getParameter("squareLength", &square_length);
  calibrator->setSquareLength(square_length);

  bool release_object;
  context.getParameter("releaseObject", &release_object, true);
  calibrator->setReleaseObject(release_object);

  bool marker;
  context.getParameter("marker", &marker, false);
  calibrator->setMarker(marker);
}

}  // namespace stereo_ugv
