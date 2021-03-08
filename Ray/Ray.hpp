#include <utility>

#ifndef TPRT_RAY_HPP
#define TPRT_RAY_HPP

#include "../Layer.hpp"
#include "../Receiver.hpp"
#include "../Source.hpp"
#include "../VelocityModel.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace ray_tracing {

enum Direction { DOWN = -1, UP = 0 };
enum WaveType { SWAVE = 0, PWAVE = 1 };

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

  std::vector<Code> ray_code; // vector of ray_code
  std::vector<std::array<float, 3>> trajectory;

  void optimizeTrajectory();
  void computeSegmentsRay();

  void generateCode(std::vector<std::array<int, 3>> ray_code);

public:
  std::vector<std::array<float, 3>> &getTrajectory() { return trajectory; }
  std::vector<Code> &getRayCodeVector() { return ray_code; }
  VelocityModel *getModel() { return velocity_model; }
  Receiver getReceiver() { return current_receiver; }
  Source getSource() { return current_source; }

  Ray(Source source, Receiver receiver, VelocityModel *_model,
      const std::vector<std::array<int, 3>>& iray_code)
      : current_source(std::move(source)),
        current_receiver(std::move(receiver)), velocity_model(_model),
        timeP(INFINITY) /*amplitudeP(1), timeS(INFINITY), amplitudeS(1)*/ {
    generateCode(iray_code);
  }

  Ray(std::vector<Source> _sources, std::vector<Receiver> _receivers,
      VelocityModel *_model, const std::vector<std::array<int, 3>>& iray_code)
      : sources(std::move(_sources)), receivers(std::move(_receivers)),
        current_source(sources[0]), current_receiver(receivers[0]),
        velocity_model(_model), timeP(INFINITY) /*amplitudeP(1),
        timeS(INFINITY)*/
  {
    current_source = sources[0];
    current_receiver = receivers[0];
    generateCode(iray_code);
  }

  void rayPolarization();

  void setTrajectory(std::vector<double> raw_trajectory) {
    std::vector<std::array<float, 3>> new_trajectory;
    new_trajectory.push_back(current_source.getLocation());
    for (unsigned long i = 1; i <= trajectory.size() - 2; i++) {
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
  }
  void computePathWithRayCode();

  rapidjson::Document toJSON();
};

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
