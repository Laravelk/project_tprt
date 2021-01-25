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

namespace ray_tracing {

enum Direction { DOWN = -1, UP = 0 };
enum WaveType { SWAVE = 0, PWAVE = 1 };
/// class for cppoptlib Solver

/// source: object of type Source
/// receiver: object of type Receiver
/// velocity_model: object of type VelocityModel
/// ray_code: for each Segment of Ray is [+1 (down) or -1 (up), depth of
/// Layer, WaveType: 0 (WaveP) or 1 (WaveS)]
///
///

struct Code {
  Code(long number, Direction dir, WaveType type)
      : direction(dir), wave_type(type), layerNumber(number) {}

  Direction direction;
  WaveType wave_type;
  long layerNumber;
};

class Ray {
private:
  std::vector<Source> sources;
  std::vector<Receiver> receivers;
  Source current_source;
  Receiver current_receiver;
  VelocityModel *velocity_model;

  float timeP;
  float amplitudeP;
  float timeS;
  float amplitudeS;

  std::vector<Code> ray_code; // vector of ray_code
  std::vector<std::array<float, 3>> trajectory;

  /// function which optimize trajectory
  /// @return nothing
  void optimizeTrajectory();

  /// function do first optimization of trajectory
  /// @return nothing
  void computeSegmentsRay();

  void generateCode(const std::vector<std::array<int, 3>> ray_code);

public:
  //  friend double myfunc(const std::vector<double> &x, std::vector<double>
  //  &grad,
  //                       void *data);

  std::vector<std::array<float, 3>> &getTrajectory() { return trajectory; }
  std::vector<Code> &getRayCodeVector() { return ray_code; }
  VelocityModel *getModel() { return velocity_model; }
  Receiver getReceiver() { return current_receiver; }
  Source getSource() { return current_source; }

  Ray(Source source, Receiver receiver, VelocityModel *_model,
      const std::vector<std::array<int, 3>> iray_code)
      : current_source(std::move(source)),
        current_receiver(std::move(receiver)), velocity_model(_model),
        timeP(INFINITY), timeS(INFINITY), amplitudeP(1), amplitudeS(1) {
    generateCode(iray_code);
  }

  Ray(std::vector<Source> _sources, std::vector<Receiver> _receivers,
      VelocityModel *_model, const std::vector<std::array<int, 3>> iray_code)
      : velocity_model(_model), timeP(INFINITY), timeS(INFINITY), amplitudeP(1),
        amplitudeS(1), sources(std::move(_sources)),
        receivers(std::move(_receivers)), current_receiver(receivers[0]),
        current_source(sources[0]) {
    current_source = sources[0];
    current_receiver = receivers[0];
    generateCode(iray_code);
  }

  void setTrajectory(std::vector<double> raw_trajectory) {
    std::vector<std::array<float, 3>> new_trajectory;
    new_trajectory.push_back(current_source.getLocation());
    for (int i = 1; i <= trajectory.size() - 2; i++) {
      new_trajectory.push_back(
          {static_cast<float>(raw_trajectory[2 * (i - 1)]),
           static_cast<float>(raw_trajectory[2 * (i - 1) + 1]),
           velocity_model->getLayer(ray_code[i - 1].layerNumber)
               ->getTop()
               ->getDepth(
                   {static_cast<float>(raw_trajectory[2 * (i - 1)]),
                    static_cast<float>(raw_trajectory[2 * (i - 1) + 1])})});
    }
    new_trajectory.push_back(current_receiver.getLocation());
    trajectory = new_trajectory;

    //    std::cerr << "trajectory" << std::endl;
    //    for (auto part : trajectory) {
    //      std::cerr << part[0] << " " << part[1] << " " << part[2] <<
    //      std::endl;
    //    }
    //    std::cerr << std::endl;
  }

  [[deprecated]] Ray(Source source, Receiver receiver, VelocityModel *_model)
      : current_source(std::move(source)), current_receiver(receiver),
        velocity_model(_model), timeP(INFINITY), timeS(INFINITY), amplitudeP(1),
        amplitudeS(1) {}

  [[deprecated]] void computePath();
  void computePathWithRayCode(); // test function for raycode

  rapidjson::Document toJSON();
};

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
