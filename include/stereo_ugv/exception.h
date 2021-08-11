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

/**
 * @brief The class for parse errors encountered during variable substitution on strings within JSON.
 */
class ParseError : public Exception
{
public:
  using Exception::Exception;
};

/**
 * @brief The class for errors due to unaccepted initialization parameters.
 */
class InvalidParameter : public Exception
{
public:
  using Exception::Exception;
};

/**
 * @brief Checks the validity of parameters by asserting that the given condition is true. Throws an InvalidParameter if
 * the assertion fails.
 * @param condition A boolean expression that is supposed to be true.
 */
#define STEREO_UGV_CHECK_PARAMETER(condition)                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!static_cast<bool>(condition))                                                                                 \
    {                                                                                                                  \
      throw stereo_ugv::InvalidParameter{ "Failed parameter check \"" #condition "\"" };                               \
    }                                                                                                                  \
  } while (false)

}  // namespace stereo_ugv

#endif  // STEREO_UGV_EXCEPTION_H
