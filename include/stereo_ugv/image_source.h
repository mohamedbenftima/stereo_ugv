/**
 * @file image_source.h
 * @brief Defines stereo_ugv::ImageSource and its subclasses.
 */

#ifndef STEREO_UGV_IMAGE_SOURCE_H
#define STEREO_UGV_IMAGE_SOURCE_H

#include "stereo_ugv/exception.h"
#include "stereo_ugv/frame_layout.h"

#include <fmt/core.h>

#include <opencv2/videoio.hpp>

#include <ros/console.h>

#include <utility>

namespace stereo_ugv
{
/**
 * @brief The base class that reads a pair of images taken by a stereo camera at a time from various sources.
 */
class ImageSource
{
public:
  static std::unique_ptr<ImageSource> create(const Context& context);

  /**
   * @brief Gets the image size. The stereo_ugv package assumes that images taken by two heads of a stereo camera have
   * the same size.
   * @return The image size.
   */
  virtual cv::Size imageSize() const = 0;

  /**
   * @brief Reads a pair of images of the next frame. The stereo_ugv package assumes that two heads of a stereo camera
   * are aligned horizontally.
   * @param left The image taken by the left head.
   * @param right The image taken by the right head.
   */
  virtual void readImages(cv::Mat* left, cv::Mat* right) = 0;
};

/**
 * @brief Types of image sources backended by cv::VideoCapture.
 */
enum class CvVideoCaptureType
{
  /**
   * @brief Images are taken by a camera device at real-time.
   */
  CAMERA,

  /**
   * @brief Images are read from a sequence of images files with a common filename pattern.
   */
  IMAGES,

  /**
   * @brief Images are extracted from frames of a video file.
   */
  VIDEO
};

/**
 * @brief Class for image sources backended by cv::VideoCapture.
 * @tparam Type The type of image source.
 */
template <CvVideoCaptureType Type>
class CvVideoCaptureImageSource : public ImageSource
{
public:
  /**
   * @brief Sets the frame layout.
   * @param layout The frame layout.
   */
  void setFrameLayout(std::unique_ptr<FrameLayout>&& layout)
  {
    frame_layout_ = std::move(layout);
  }

  /**
   * @brief Opens the video capture at the given filename.
   * @warning The frame layout must be set before calling this method, because this method relies on the frame layout to
   * provide the frame size used to initialize the image source.
   * @param filname If the type is CAMERA, it is the name of the character device file. If the type if IMAGES, it is the
   * pattern of the image sequence filenames. If the type if VIDEO, it is the name of the video file.
   */
  void openVideoCapture(const std::string& filename)
  {
    if (!video_capture_.open(filename))
    {
      throw RuntimeError{ fmt::format(R"(VideoCapture could not open "{}")", filename) };
    }

    if constexpr (Type != CvVideoCaptureType::IMAGES)
    {
      const auto expected{ frame_layout_->frameSize() };

      if constexpr (Type == CvVideoCaptureType::CAMERA)
      {
        if (!video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, expected.width) ||
            !video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, expected.height))
        {
          throw RuntimeError{ fmt::format("Camera does not support the frame size {}x{}", expected.width,
                                          expected.height) };
        }
      }

      const auto actual_width{ video_capture_.get(cv::CAP_PROP_FRAME_WIDTH) };
      const auto actual_height{ video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT) };
      if (actual_width != expected.width || actual_height != expected.height)
      {
        throw RuntimeError{ fmt::format("Frame size of the video capture {}x{} does not match the expected size {}x{}",
                                        actual_width, actual_height, expected.width, expected.height) };
      }
    }
  }

  virtual cv::Size imageSize() const override
  {
    return frame_layout_->imageSize();
  }

  virtual void readImages(cv::Mat* left, cv::Mat* right) override
  {
    readFrame();
    frame_layout_->extractImages(frame_, left, right);
  }

private:
  void readFrame()
  {
    if constexpr (Type == CvVideoCaptureType::IMAGES)
    {
      const auto expected{ frame_layout_->frameSize() };
      while (video_capture_.read(frame_))
      {
        const auto actual{ frame_.size() };
        if (expected == actual)
        {
          return;
        }
        ROS_WARN_STREAM(fmt::format("Skipping one {}x{} frame because the expected frame size is {}x{}", actual.width,
                                    actual.height, expected.width, expected.height));
      }
      throw OutOfFrames{ "All image files in the sequence have been read" };
    }
    else if (!video_capture_.read(frame_))
    {
      if constexpr (Type == CvVideoCaptureType::CAMERA)
      {
        throw OutOfFrames{ "Could not read images because the camera was disconnected" };
      }
      if constexpr (Type == CvVideoCaptureType::VIDEO)
      {
        throw OutOfFrames{ "Could not read images because the end of video was encountered" };
      }
    }
  }

  std::unique_ptr<FrameLayout> frame_layout_;
  cv::VideoCapture video_capture_;
  mutable cv::Mat frame_;
};

std::string findCameraCharacterDeviceFile(const std::string& device_name_prefix);

/**
 * @brief Initializes a CvVideoCaptureImageSource<Type>.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   "frameLayout": <FrameLayout>,
 *   "type": <string>,
 *   ...
 * }
 * @code
 * where ... means type-specific entries. For "camera":
 * @code
 * {
 *   ...
 *   "deviceNamePrefix": <string>
 * }
 * @code
 * For "images":
 * @code
 * {
 *   ...
 *   "filenamePattern": <string>
 * }
 * @code
 * For "video":
 * @code
 * {
 *   ...
 *   "filename": <string>
 * }
 * @code
 * @param source The image source to be initialized.
 * @param context The context.
 */
template <CvVideoCaptureType Type>
inline void initialize(CvVideoCaptureImageSource<Type>* source, const Context& context)
{
  source->setFrameLayout(FrameLayout::create({ context, "frameLayout" }));

  std::string filename;
  if constexpr (Type == CvVideoCaptureType::CAMERA)
  {
    std::string device_name_prefix;
    context.getParameter("deviceNamePrefix", &device_name_prefix);
    filename = findCameraCharacterDeviceFile(device_name_prefix);
  }
  if constexpr (Type == CvVideoCaptureType::IMAGES)
  {
    context.getParameter("filenamePattern", &filename);
  }
  if constexpr (Type == CvVideoCaptureType::VIDEO)
  {
    context.getParameter("filename", &filename);
  }
  source->openVideoCapture(filename);
}

}  // namespace stereo_ugv

#endif  // STEREO_UGV_IMAGE_SOURCE_H
