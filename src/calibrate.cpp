#include "stereo_ugv/lens_calibrator.h"

using stereo_ugv::Context;
using stereo_ugv::LensCalibrator;

using stereo_ugv::initializeContextArguments;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrator");
  ros::NodeHandle node_handle{ "~" };

  nlohmann::json parameter_json;
  std::unordered_map<std::string, std::string> variable_map;
  initializeContextArguments(node_handle, &parameter_json, &variable_map);

  image_transport::ImageTransport image_transport{ node_handle };
  const Context context{ &parameter_json, &variable_map, &image_transport };

  auto calibrator{ LensCalibrator::create({ context, "lensCalibrator" }) };
  calibrator->calculateCoefficients();
  calibrator->saveCoefficients();

  return 0;
}
