#include "stereo_ugv/exception.h"

#include <algorithm>
#include <cstring>

namespace stereo_ugv
{
/**
 * @brief Creates an exception.
 * @param message The explanatory string. It should be UTF-8 encoded and null-terminated.
 */
Exception::Exception(const char* message)
{
  const auto size{ std::strlen(message) + 1 };
  message_ = std::shared_ptr<char[]>{ new char[size] };
  std::copy_n(message, size, message_.get());
}

/**
 * @brief Creates an exception.
 * @param message The UTF-8 encoded explanatory string.
 */
Exception::Exception(const std::string& message) : Exception{ message.c_str() }
{
}

/**
 * @brief Gets the explanatory string.
 * @return The explanatory string.
 */
const char* Exception::what() const noexcept
{
  return message_.get();
}

}  // namespace stereo_ugv
