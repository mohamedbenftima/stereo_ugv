/**
 * @file exception.h
 * @brief Defines stereo_ugv::Exception and its subclasses.
 */

#ifndef STEREO_UGV_EXCEPTION_H
#define STEREO_UGV_EXCEPTION_H

#include <exception>
#include <memory>
#include <string>

/**
 * @brief The namespace for public classes and functions of the stereo_ugv package.
 */
namespace stereo_ugv
{
/**
 * @brief The base class for exceptions.
 */
class Exception : public std::exception
{
public:
  Exception(const char* message);

  Exception(const std::string& message);

  virtual const char* what() const noexcept override;

private:
  std::shared_ptr<char[]> message_;
};

}  // namespace stereo_ugv

#endif  // STEREO_UGV_EXCEPTION_H
