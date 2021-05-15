#include <utility>

#ifndef TPRT_RAY_HPP
#define TPRT_RAY_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "../Layer.hpp"
#include "../Receiver.hpp"
#include "../Source.hpp"
#include "../VelocityModel.hpp"
#include "WaveType.h"

using namespace Eigen;

namespace ray_tracing {
enum Direction { DOWN = -1, UP = 0 };

struct Code {
  Code(long number, Direction dir, WaveType type)
      : direction(dir), wave_type(type), layerNumber(number) {}

  Direction direction;
  WaveType wave_type;
  long layerNumber;
};

class Ray {
private:
  Source source;
  Receiver receiver;
  VelocityModel *velocity_model;
  const WaveType waveType = WaveType::PWave; // TODO: changed

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
  Receiver getReceiver() { return receiver; }
  Source getSource() { return source; }

  std::vector<Eigen::Vector3f> getVectors() {
    std::vector<Eigen::Vector3f> vectors;
    for (int i = 0; i < trajectory.size() - 1; i++) {
        Eigen::Vector3f vector;
        for (int j = 0; j < 3; j++) {
            vector(j) = trajectory[i][j] - trajectory[i + 1][j];
        }

        vector = vector.normalized().cwiseAbs();
        vectors.emplace_back(vector);
    }
    return vectors;
  }

  std::vector<float> getVels(int count) {
      std::vector<float> vels;
      if (WaveType::PWave == waveType) {
          for (int i = 0; i < count; i++) {
              vels.push_back(velocity_model->getLayer(i)->getVp());
          }
      } else {
          for (int i = 0; i < count; i++) {
              vels.push_back(velocity_model->getLayer(i)->getVs());
          }
      }
      return vels;
  }

  Ray(Source source, Receiver receiver, VelocityModel *_model,
      const std::vector<std::array<int, 3>>& iray_code)
      : source(std::move(source)),
        receiver(std::move(receiver)), velocity_model(_model),
        timeP(INFINITY) /*amplitudeP(1), timeS(INFINITY), amplitudeS(1)*/ {
    generateCode(iray_code);
  }

  Ray(std::vector<Source> _sources, std::vector<Receiver> _receivers,
      VelocityModel *_model, const std::vector<std::array<int, 3>>& iray_code)
      :
        source(source), receiver(receiver),
        velocity_model(_model), timeP(INFINITY) /*amplitudeP(1),
        timeS(INFINITY)*/
  {
    source = source;
    receiver = receiver;
    generateCode(iray_code);
  }

    Vector3f rayPolarization();
    void computeAmplitude();
    void raySpreading();
    Eigen::Vector3f rt_coeffs_iso(Vector3f, Vector3f, Vector3f, Vector3f, int, Layer::Properties, Layer::Properties);

  void setTrajectory(std::vector<double> raw_trajectory) {
    std::vector<std::array<float, 3>> new_trajectory;
    new_trajectory.push_back(source.getLocation());
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
    new_trajectory.push_back(receiver.getLocation());
    trajectory = new_trajectory;
  }

  std::vector<float> getDistance();
  void computePathWithRayCode();
  rapidjson::Document toJSON();
};

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
