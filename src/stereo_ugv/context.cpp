#include "stereo_ugv/context.h"
#include "stereo_ugv/exception.h"

#include <fmt/core.h>

#include <ros/package.h>

#include <fstream>
#include <stack>

namespace stereo_ugv
{
/**
 * @brief Initializes arguments to the Context constructor from parameters of a node handle.
 * @details Parameters for object initialization are loaded from a JSON file. The path to this file should be specified
 * at the "parameter_file" key of the node handle's private namespace. The parameter map currently contains only a few
 * pre-defined entries (i.e. "configurationFolder" and "dataFolder"), but it may allow being loaded from the parameter
 * server in the future.
 * @param node The node handle.
 * @param parameter_json A JSON object storing initialization parameters. See Context::Context for details.
 * @param variable_map A map for performing variable substitution. See Context::Context for details.
 */
void initializeContextArguments(const ros::NodeHandle& node_handle, nlohmann::json* parameter_json,
                                std::unordered_map<std::string, std::string>* variable_map)
{
  std::string parameter_file;
  if (!node_handle.getParam("parameter_file", parameter_file))
  {
    throw RuntimeError{ "The parameter file is not specified" };
  }

  std::ifstream stream{ parameter_file };
  if (!stream.is_open())
  {
    throw RuntimeError{ fmt::format(R"(Could not open "{}")", parameter_file) };
  }
  *parameter_json = nlohmann::json::parse(stream);

  variable_map->clear();
  ros::M_string map;
  node_handle.getParam("variables", map);
  for (const auto& [key, value] : map)
  {
    variable_map->emplace(key, value);
  }
}

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
 * @brief Creates a new context based on the given original context, except that the new underlying JSON is the child of
 * the original underlying JSON at the given key.
 * @param other The original context.
 * @param key The key of the original underlying JSON object.
 */
Context::Context(const Context& other, const std::string& key)
  : Context{ &other.parameter_json_->at(key), other.variable_map_, other.image_transport_ }
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
 * @endcode
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
 * @endcode
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

/**
 * @brief Opens the internal context whose variable JSON is stored in a separate file.
 * @details A separate file that stores the variable JSON should be referenced as follows:
 * @code
 * {
 *   "type": "file",
 *   "filename": <string>
 * }
 * @endcode
 * @param json The variable JSON file to be opened. It will not be assigned if no variable JSON file is referenced.
 * @param context The context on the surface.
 * @return If the operation was successful, i.e. a separate file is referenced that stores the variable JSON, returns a
 * new context created with this internal JSON. Otherwise, returns the original context passed into this function.
 */
Context openInternalContext(nlohmann::json* json, const Context& context)
{
  const auto surface_json{ &context.parameterJSON() };

  std::string type;
  surface_json->at("type").get_to(type);
  if (type != "file")
  {
    return context;
  }

  std::string filename;
  surface_json->at("filename").get_to(filename);
  filename = substituteVariables(filename, context.variableMap());
  std::ifstream stream{ filename };
  if (!stream.is_open())
  {
    throw RuntimeError{ fmt::format(R"(Could not open "{}")", filename) };
  }
  *json = nlohmann::json::parse(stream);

  while (json->is_object() && json->at("type").get_to(type) == "file")
  {
    json->at("filename").get_to(filename);
    filename = substituteVariables(filename, context.variableMap());
    stream.open(filename);
    if (!stream.is_open())
    {
      throw RuntimeError{ fmt::format(R"(Could not open "{}")", filename) };
    }
    *json = nlohmann::json::parse(stream);
  }

  return { json, &context.variableMap(), &context.imageTransport() };
}

}  // namespace stereo_ugv
