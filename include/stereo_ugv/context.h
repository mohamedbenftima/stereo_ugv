/**
 * @file context.h
 * @brief Defines stereo_ugv::Context and initialization functions of several common types.
 */

#ifndef STEREO_UGV_CONTEXT_H
#define STEREO_UGV_CONTEXT_H

#include <nlohmann/json.hpp>

#include <opencv2/core/types.hpp>

#include <image_transport/image_transport.h>

#include <string>
#include <unordered_map>

namespace stereo_ugv
{
/**
 * @brief The class for initializing objects from parameters stored in JSON files.
 */
class Context
{
public:
  Context(const nlohmann::json* parameter_json, const std::unordered_map<std::string, std::string>* variable_map,
          image_transport::ImageTransport* image_transport) noexcept;

  Context(const Context& other, const std::string& key);

  const nlohmann::json& parameterJSON() const noexcept;

  const std::unordered_map<std::string, std::string>& variableMap() const noexcept;

  image_transport::ImageTransport& imageTransport() const noexcept;

  /**
   * @brief Initializes the parameter of the given key.
   * @tparam T The type of the parameter to be initialized. The initialize(T*, const Context&) function is required.
   * The initialization function should be defined in the same namespace as type T so that it can be found using
   * argument-dependent lookup (ADL).
   * @param key The key at which the initialization parameters are stored.
   * @param parameter The parameter to be initialized.
   */
  template <typename T>
  void getParameter(const std::string& key, T* parameter) const
  {
    initialize(parameter, Context{ *this, key });
  }

  /**
   * @brief Initializes the parameter of the given key, or assigns it with the default value if the key does not exist.
   * @tparam T The type of the parameter to be initialized. The initialize(T*, const Context&) function is required.
   * The initialization function should be defined in the same namespace as type T so that it can be found using
   * argument-dependent lookup (ADL).
   * @param key The key at which the initialization parameters are stored.
   * @param parameter The parameter to be initialized.
   * @param default_value The default value used when the given key does not exist.
   */
  template <typename T>
  void getParameter(const std::string& key, T* parameter, const T& default_value) const
  {
    if (parameter_json_->contains(key))
    {
      getParameter<T>(key, parameter);
    }
    else
    {
      *parameter = default_value;
    }
  }

private:
  const nlohmann::json* parameter_json_;
  const std::unordered_map<std::string, std::string>* variable_map_;
  image_transport::ImageTransport* image_transport_;
};

/**
 * @brief Initializes a parameter.
 * @tparam T The type of the parameter to be initialized. It should be directly representable by the JSON format, e.g.
 * boolean, number, string, etc. Otherwise, a custom overload of this function should be defined and used.
 * @param parameter The parameter to be initialized.
 * @param context The context.
 */
template <typename T>
inline void initialize(T* parameter, const Context& context)
{
  context.parameterJSON().get_to(*parameter);
}

void initialize(std::string* string, const Context& context);

void initialize(cv::Size* size, const Context& context);

void initialize(cv::TermCriteria* criteria, const Context& context);

Context openInternalContext(nlohmann::json* json, const Context& context);

}  // namespace stereo_ugv

#endif  // STEREO_UGV_CONTEXT_H
