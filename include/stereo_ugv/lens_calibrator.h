/**
 * @file lens_calibrator.h
 * @brief Defines stereo_ugv::LensCalibrator and its subclasses.
 */

#ifndef STEREO_UGV_LENS_CALIBRATOR_H
#define STEREO_UGV_LENS_CALIBRATOR_H

#include "image_source.h"

#include <cstddef>
#include <vector>

namespace stereo_ugv
{
/**
 * @brief The structure of undistortion coefficients of a stereo camera.
 */
struct LensCoefficients
{
  /**
   * @brief 3x3 camera matrices of the left and the right heads, respectively.
   */
  cv::Mat camera_matrices[2];

  /**
   * @brief 5x1 distortion coefficients of the left and the right heads, respectively.
   * @details The coefficients are ordered as (k1, k2, p1, p2, k3), where k describes radial distortions, and p
   * describes tangential distortions.
   */
  cv::Mat distortion_coefficients[2];

  /**
   * @brief The rotation matrix from the left head to the right head.
   */
  cv::Mat rotation_matrix;

  /**
   * @brief The translation vector from the left head to the right head.
   */
  cv::Mat translation_vector;
};

/**
 * @brief The base class for calculating and saving undistortion coefficients of a stereo camera.
 */
class LensCalibrator
{
public:
  const LensCoefficients& coefficients() const noexcept;

  void setImageSource(std::unique_ptr<ImageSource>&& source);

  void setMaxImageCount(std::size_t count);

  void setUndistortionFilename(std::string filename);

  void calculateCoefficients();

  void saveCoefficients();

protected:
  /**
   * @brief Detects feature points in the given image.
   * @details The number of detected points must be constant, and these 2D points should corresponds to fixed points on
   * a roughly planar rigid body in 3D space.
   * @param image The image.
   * @param points The feature points to be detected. It will not be assigned if no feature points were detected.
   * @return Whether feature points are detected.
   */
  virtual bool detectImagePoints(const cv::Mat& image, std::vector<cv::Point2f>* points) const = 0;

  /**
   * @brief Creates the 3D points corresponding to the detected 2D points in images.
   * @return A vector of 3D points.
   */
  virtual std::vector<cv::Point3f> createObjectPoints() const = 0;

  /**
   * @brief Gets the index of the fixed point if the method of releasing object is used.
   * @return If the traditional calibration method is used, returns 0. Otherwise if the method of releasing object is
   * used, return the index of the fixed point which is between 1 and number_of_points - 1.
   */
  virtual int indexOfFixedPoint() const = 0;

private:
  void detectFramePoints(std::vector<std::vector<cv::Point2f>> image_points[2],
                         std::vector<std::vector<cv::Point2f>> common_image_points[2]) const;

  std::unique_ptr<ImageSource> image_source_;
  std::size_t max_image_count_;
  std::string undistortion_filename_;
  LensCoefficients coefficients_;
  mutable cv::Mat images_[2];
};

void initialize(LensCalibrator* calibrator, const Context& context);

/**
 * @brief The class for calibrating a stereo camera using a chessboard pattern.
 */
class ChessboardLensCalibrator : public LensCalibrator
{
public:
  void setPatternSize(cv::Size size);

  void setSquareLength(float length);

  void setReleaseObject(bool enabled);

  void setMarker(bool enabled);

protected:
  virtual bool detectImagePoints(const cv::Mat& image, std::vector<cv::Point2f>* points) const override;

  virtual std::vector<cv::Point3f> createObjectPoints() const override;

  virtual int indexOfFixedPoint() const noexcept override;

private:
  cv::Size pattern_size_;
  float square_length_;
  bool release_object_;
  bool marker_;
};

void initialize(ChessboardLensCalibrator* calibrator, const Context& context);

}  // namespace stereo_ugv

#endif  // STEREO_UGV_LENS_CALIBRATOR_H
