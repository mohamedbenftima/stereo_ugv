/**
 * @file frame_layout.h
 * @brief Defines stereo_ugv::FrameLayout and its subclasses.
 */

#ifndef STEREO_UGV_FRAME_LAYOUT_H
#define STEREO_UGV_FRAME_LAYOUT_H

#include "stereo_ugv/context.h"

#include <opencv2/core/mat.hpp>

#include <memory>

namespace stereo_ugv
{
/**
 * @brief The base class that defines how images taken by two heads of a stereo camera are positioned in the same frame.
 */
class FrameLayout
{
public:
  static std::unique_ptr<FrameLayout> create(const Context& context);

  /**
   * @brief Gets the frame size.
   * @return The frame size.
   */
  virtual cv::Size frameSize() const = 0;

  /**
   * @brief Gets the image size. The stereo_ugv package assumes that images taken by two heads of a stereo camera have
   * the same size.
   * @return The image size.
   */
  virtual cv::Size imageSize() const = 0;

  /**
   * @brief Extracts images from the given frame. The stereo_ugv package assumes that two heads of a stereo camera are
   * aligned horizontally.
   * @param frame The frame.
   * @param left The left image to be extracted.
   * @param right The right image to be extracted.
   */
  virtual void extractImages(const cv::Mat& frame, cv::Mat* left, cv::Mat* right) const = 0;
};

/**
 * @brief The class of frame layout for HPS-3D160 cameras.
 */
class HPS3D160FrameLayout : public FrameLayout
{
public:
  void setImageSize(cv::Size size);

  virtual cv::Size frameSize() const noexcept override;

  virtual cv::Size imageSize() const noexcept override;

  virtual void extractImages(const cv::Mat& frame, cv::Mat* left, cv::Mat* right) const override;

private:
  cv::Size image_size_;
};

void initialize(HPS3D160FrameLayout* layout, const Context& context);

}  // namespace stereo_ugv

#endif  // STEREO_UGV_FRAME_LAYOUT_H
