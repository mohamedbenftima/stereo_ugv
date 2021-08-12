#include "stereo_ugv/image_source.h"

namespace stereo_ugv
{
/**
 * @brief Creates an ImageSource.
 * @details The function determines the concrete subclass of the layout to be created by looking up the "type" key, and
 * calls the corresponding initialization function. Currently, "camera", "images" and "video" are supported.
 * @param context The context containing initialization parameters.
 * @return A unique pointer to the created image source.
 */
std::unique_ptr<ImageSource> ImageSource::create(const Context& context)
{
  nlohmann::json internal_json;
  const auto internal_context{ openInternalContext(&internal_json, context) };

  std::string type;
  internal_context.getParameter("type", &type);

  if (type == "camera")
  {
    auto source{ std::make_unique<CvVideoCaptureImageSource<CvVideoCaptureType::CAMERA>>() };
    initialize(source.get(), context);
    return source;
  }

  if (type == "images")
  {
    auto source{ std::make_unique<CvVideoCaptureImageSource<CvVideoCaptureType::IMAGES>>() };
    initialize(source.get(), context);
    return source;
  }

  if (type == "video")
  {
    auto source{ std::make_unique<CvVideoCaptureImageSource<CvVideoCaptureType::VIDEO>>() };
    initialize(source.get(), context);
    return source;
  }

  throw InvalidParameter{ fmt::format(R"(Unknown ImageSource type "{}")", type) };
}

/**
 * @brief Finds the character device file of a camera device given the prefix of its device name.
 * @warning This function has not been implemented, and calling it will always cause a RuntimeError.
 * @param device_name_prefix The prefix of its device name. Due to the limits of the V4L2 API, device names longer than
 * 31 characters are truncated. Therefore, the length of the given prefix should not be longer than 31 characters.
 * Otherwise, no matching camera can be found.
 * @return The full path to the character device file. If multiple matching cameras are detected, the function issues a
 * warning and returns any one among them.
 */
std::string findCameraCharacterDeviceFile(const std::string&)
{
  throw RuntimeError{ "This function has not been implemented" };
}

}  // namespace stereo_ugv
