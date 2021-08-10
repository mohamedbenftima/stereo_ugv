#include "stereo_ugv/context.h"
#include "stereo_ugv/exception.h"

#include <fmt/core.h>

#include <stack>

namespace stereo_ugv
{
/**
 * @brief Creates a context.
 * @param parameter_json A JSON object that stores parameters for object initialization.
 * @param variable_map A key-value map for performing variable substitution on strings within JSON.
 * @param image_transport An image transport for creating image publishers.
 */
Context::Context(const nlohmann::json* parameter_json, const std::unordered_map<std::string, std::string>* variable_map,
                 image_transport::ImageTransport* image_transport) noexcept
  : parameter_json_{ parameter_json }, variable_map_{ variable_map }, image_transport_{ image_transport }
{
}

/**
 * @brief Gets the JSON object that stores parameters for object initialization.
 * @return The JSON object.
 */
const nlohmann::json& Context::parameterJSON() const noexcept
{
  return *parameter_json_;
}

/**
 * @brief Gets the map for performing variable substitution on strings.
 * @return The map.
 */
const std::unordered_map<std::string, std::string>& Context::variableMap() const noexcept
{
  return *variable_map_;
}

/**
 * @brief Gets the image transport for creating image publishers.
 * @return The image transport.
 */
image_transport::ImageTransport& Context::imageTransport() const noexcept
{
  return *image_transport_;
}

static std::string substituteVariables(const std::string& string,
                                       const std::unordered_map<std::string, std::string>& variable_map)
{
  std::string result;
  auto was_dollar{ false };
  std::stack<std::string::size_type> start_indices;

  for (auto it{ string.begin() }; it != string.end(); ++it)
  {
    switch (*it)
    {
      case '\\':
        if (++it == string.end())
        {
          throw ParseError{ fmt::format(R"(Escape sequence encounters the end of string in "{}")", string) };
        }
        result.push_back(*it);
        was_dollar = false;
        break;

      case '$':
        result.push_back('$');
        was_dollar = true;
        break;

      case '{':
        if (was_dollar)
        {
          result.pop_back();
          start_indices.push(result.length());
        }
        else
        {
          result.push_back('{');
        }
        was_dollar = false;
        break;

      case '}':
        if (start_indices.empty())
        {
          result.push_back('}');
        }
        else
        {
          const auto name{ result.substr(start_indices.top()) };
          const auto found{ variable_map.find(name) };
          if (found == variable_map.end())
          {
            throw ParseError{ fmt::format(R"(Unknown variable name "{}" in "{}")", name, string) };
          }
          result.resize(start_indices.top());
          start_indices.pop();
          result += found->second;
        }
        was_dollar = false;
        break;

      default:
        result.push_back(*it);
        was_dollar = false;
    }
  }

  if (!start_indices.empty())
  {
    throw ParseError(fmt::format(R"(Mismatched braces in "{}")", string));
  }
  return result;
}

/**
 * @brief Initializes an std::string and performs variable substitution.
 * @details The variable names embedded the original string within JSON are substituted into their corresponding values
 * using the internal variable map. Variable names are marked by a pair of enclosing curly braces preceded by a dollar
 * sign, e.g. "${dataFolder}/calibration". Variable substitution can be escaped by adding a backslash before either the
 * dollar sign or the opening brace. This can be helpful for expressing file paths with environmental variables, e.g.
 * "\${HOME}/catkin_ws".
 * @param string The string to be initialized.
 * @param context The context.
 */
void initialize(std::string* string, const Context& context)
{
  context.parameterJSON().get_to(*string);
  *string = substituteVariables(*string, context.variableMap());
}

/**
 * @brief Initializes a cv::Size.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   "width": <integer>,
 *   "height": <integer>
 * }
 * @code
 * @param size The size to be initialized.
 * @param context The context.
 */
void initialize(cv::Size* size, const Context& context)
{
  context.getParameter("width", &size->width);
  context.getParameter("height", &size->height);
}

/**
 * @brief Initializes a cv::TermCriteria.
 * @details The underlying JSON object should look like:
 * @code
 * {
 *   "maxCount": <null | integer>,
 *   "epsilon": <null | number>
 * }
 * @code
 * @param criteria The criteria to be initialized.
 * @param context The context.
 */
void initialize(cv::TermCriteria* criteria, const Context& context)
{
  auto& json{ context.parameterJSON() };
  criteria->type = 0;
  if (auto& max_count{ json.at("maxCount") }; !max_count.is_null())
  {
    max_count.get_to(criteria->maxCount);
    criteria->type |= cv::TermCriteria::COUNT;
  }
  if (auto& epsilon{ json.at("epsilon") }; !epsilon.is_null())
  {
    epsilon.get_to(criteria->epsilon);
    criteria->type |= cv::TermCriteria::EPS;
  }
}

}  // namespace stereo_ugv
