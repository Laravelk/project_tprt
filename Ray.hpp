#include <utility>

#ifndef TPRT_RAY_HPP
#define TPRT_RAY_HPP

#include "Layer.hpp"
#include "Receiver.hpp"
#include "Segment.hpp"
#include "Source.hpp"
#include "VelocityModel.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <vnl/vnl_cost_function.h>

namespace ray_tracing {
enum Direction { DOWN = -1, UP = 1 };
enum WaveType { SWAVE = 0, PWAVE = 1 };

class Ray {
private:
  Source source;
  Receiver receiver;
  VelocityModel velocity_model;
  float timeP;
  float amplitudeP;
  float timeS;
  float amplitudeS;

  struct Code {
    Code(long number, Direction dir, WaveType type)
        : direction(dir), wave_type(type), layerNumber(number) {}

    Direction direction;
    WaveType wave_type;
    long layerNumber;
  };
  std::vector<Code> ray_code;                   // vector of ray_code
  std::vector<std::array<float, 3>> trajectory; // vector of points

  // buff with trajectory
  // horizon* -> unique_ptr

  Layer getLocationLayer(std::array<float, 3> location);

  //  class cost_function : public vnl_cost_function {
  //    Ray *ray;
  //    enum WaveType type;

  //  public:
  //    cost_function(Ray *ray, int number_of_unknowns, enum WaveType type);

  //    double f(vnl_vector<double> const &x) override;

  //    /*
  //        @return ray_code: [layer_number; up or down (-1 or 1); ]
  //    */
  //    static std::vector<std::array<int, 3>>
  //    fromJSON(const rapidjson::Value &doc);
  //  };

  // void optimizeTrajectory();
  void computeSegments();
  void computeSegmentsRay();

  void generateCode(const std::vector<std::array<int, 3>> ray_code);
  /*
   * source: object of type Source
   * receiver: object of type Receiver
   * velocity_model: object of type VelocityModel
   * ray_code: for each Segment of Ray is [+1 (down) or -1 (up), depth of Layer,
   * WaveType: 0 (WaveP) or 1 (WaveS)]
   */
public:
  Ray(Source source, Receiver receiver, VelocityModel velocity_model,
      const std::vector<std::array<int, 3>> iray_code)
      : source(std::move(source)), receiver(std::move(receiver)),
        velocity_model(std::move(velocity_model)) {
    generateCode(iray_code);
    timeP = INFINITY;
    timeS = INFINITY;
    amplitudeP = 1;
    amplitudeS = 1;
  }

  Ray(Source source, Receiver receiver, VelocityModel velocity_model)
      : source(std::move(source)), receiver(receiver),
        velocity_model(velocity_model) {
    timeP = INFINITY;
    timeS = INFINITY;
    amplitudeP = 1;
    amplitudeS = 1;
  }

  void computePath();
  void computePathWithRayCode(); // test function for raycode

  rapidjson::Document toJSON();
};

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
