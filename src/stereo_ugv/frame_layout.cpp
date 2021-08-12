#include "stereo_ugv/exception.h"
#include "stereo_ugv/frame_layout.h"

#include <fmt/core.h>

#include <ros/assert.h>

namespace stereo_ugv
{
/**
 * @brief Creates a FrameLayout.
 * @details The function determines the concrete subclass of the layout to be created by looking up the "type" key, and
 * calls the corresponding initialization function. Currently, only "HPS-3D160" is supported.
 * @param context The context containing initialization parameters.
 * @return A unique pointer to the created layout.
 */
std::unique_ptr<FrameLayout> FrameLayout::create(const Context& context)
{
  nlohmann::json internal_json;
  const auto internal_context{ openInternalContext(&internal_json, context) };

  std::string type;
  internal_context.getParameter("type", &type);

  if (type == "HPS-3D160")
  {
    auto layout{ std::make_unique<HPS3D160FrameLayout>() };
    initialize(layout.get(), context);
    return layout;
  }

  throw InvalidParameter{ fmt::format(R"(Unknown FrameLayout type "{}")", type) };
}

/**
 * @brief Sets the image size.
 * @param size The image size.
 */
void HPS3D160FrameLayout::setImageSize(cv::Size size)
{
  STEREO_UGV_CHECK_PARAMETER(size.width > 0 && size.height > 0);
  image_size_ = size;
}

cv::Size HPS3D160FrameLayout::frameSize() const noexcept
{
  return { 2 * image_size_.width, image_size_.height };
}

cv::Size HPS3D160FrameLayout::imageSize() const noexcept
{
  return image_size_;
}

void HPS3D160FrameLayout::extractImages(const cv::Mat& frame, cv::Mat* left, cv::Mat* right) const
{
  auto frame_size{ frame.size() };
  ROS_ASSERT(frame_size == frameSize());
  *left = frame(cv::Range::all(), { image_size_.width, frame_size.width });
  *right = frame(cv::Range::all(), { 0, image_size_.width });
}

/**
 * @brief Initializes an HPS3D160FrameLayout.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   "imageSize": <cv::Size>
 * }
 * @endcode
 * @param layout The layout to be initialized.
 * @param context The context.
 */
void initialize(HPS3D160FrameLayout* layout, const Context& context)
{
  cv::Size image_size;
  context.getParameter("imageSize", &image_size);
  layout->setImageSize(image_size);
}

}  // namespace stereo_ugv
