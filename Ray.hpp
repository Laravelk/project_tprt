#include <utility>

#ifndef TPRT_RAY_HPP
#define TPRT_RAY_HPP

#include "Layer.hpp"
#include "Receiver.hpp"
#include "Segment.hpp"
#include "Source.hpp"
#include "VelocityModel.hpp"
#include "cppoptlib/meta.h"
#include "cppoptlib/problem.h"
#include "cppoptlib/solver/bfgssolver.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace cppoptlib;

namespace ray_tracing {
enum Direction { DOWN = -1, UP = 0 };
enum WaveType { SWAVE = 0, PWAVE = 1 };
/// class for cppoptlib Solver

/// source: object of type Source
/// receiver: object of type Receiver
/// velocity_model: object of type VelocityModel
/// ray_code: for each Segment of Ray is [+1 (down) or -1 (up), depth of Layer,
/// WaveType: 0 (WaveP) or 1 (WaveS)]
class Ray {
private:
  class MyProblem : public Problem<double> {
  private:
    VelocityModel *model = nullptr;
    Ray &ray;

    /// source_location: source_cord
    /// receiver_location: receiver_cord
    /// iteration: i in ray_code
    double calculation_part_time(std::array<float, 3> source_location,
                                 std::array<float, 3> receiver_location,
                                 int iteration) {

      std::array<float, 3> vec{receiver_location[0] - source_location[0],
                               receiver_location[1] - source_location[1],
                               receiver_location[2] - source_location[2]};

      float norm_vec =
          sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

      auto layer =
          ray.velocity_model->getLayer(ray.ray_code.at(iteration).layerNumber);

      return norm_vec / layer->getVp();
    }

  public:
    using typename Problem<double>::TVector;

    MyProblem(Ray &_ray) : ray(_ray) {}

    double value(const TVector &x) override {
      double time = 0;
      int number_of_unknowns = ray.ray_code.size();

      // number_of_unknowns - 2 because the last element of trajectory is
      // receiver
      for (int i = 1; i < number_of_unknowns - 2; i++) {
        ray.trajectory[i] = {
            static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
            ray.velocity_model->getLayer(ray.ray_code[i + 1].layerNumber)
                ->getTop()
                ->getDepth({static_cast<float>(x[2 * i]),
                            static_cast<float>(x[2 * i + 1])})};
      }
      std::cerr
          << "after calculation new x, y, z positions \n"; // TODO: delete or
                                                           // #ifdef as debug

      std::array<float, 3> source_location = ray.trajectory[0];

      for (int i = 1; i < ray.trajectory.size() - 1; i++) {
        std::cerr << "The second sysle::value::MyProblem::Iteration " << i
                  << " of " << ray.trajectory.size()
                  << "\n"; // TODO: remove or #ifdef debug
        std::array<float, 3> receiver_location = ray.trajectory.at(i);
        time += calculation_part_time(source_location, receiver_location, i);
        source_location = receiver_location;
      }

      time +=
          calculation_part_time(source_location, ray.receiver.getLocation(), 0);

      std::cerr << "return time " << time
                << "\n"; // TODO: delete or #ifdef debug
      return time;
    }

    void gradient(const TVector &x, TVector &grad) override {
    } /* TODO: check python realization */
  };

  Source source;
  Receiver receiver;
  VelocityModel
      *velocity_model; /* TODO: -> * (std::uniuq_ptr in main) or shared */
  float timeP;
  float amplitudeP;
  float timeS;
  float amplitudeS;
  std::unique_ptr<MyProblem> problem; /* TODO: remove unique_ptr */

  struct Code {
    Code(long number, Direction dir, WaveType type)
        : direction(dir), wave_type(type), layerNumber(number) {}

    Direction direction;
    WaveType wave_type;
    long layerNumber;
  };
  std::vector<Code> ray_code; // vector of ray_code

  std::vector<std::array<float, 3>> trajectory; // vector of points

  /// function which optimize trajectory
  /// @return nothing
  void optimizeTrajectory();

  /// function do first optimization of trajectory
  /// @return nothing
  void computeSegmentsRay();

  void generateCode(const std::vector<std::array<int, 3>> ray_code);

public:
  Ray(Source source, Receiver receiver, VelocityModel *_model,
      const std::vector<std::array<int, 3>> iray_code)
      : source(std::move(source)), receiver(std::move(receiver)),
        velocity_model(_model), timeP(INFINITY), timeS(INFINITY), amplitudeP(1),
        amplitudeS(1) {
    generateCode(iray_code);
    problem = std::make_unique<MyProblem>(*this);
  }

  [[deprecated]] Ray(Source source, Receiver receiver, VelocityModel *_model)
      : source(std::move(source)), receiver(receiver), velocity_model(_model),
        timeP(INFINITY), timeS(INFINITY), amplitudeP(1), amplitudeS(1) {
    problem = std::make_unique<MyProblem>(*this);
  }

  [[deprecated]] void computePath();
  void computePathWithRayCode(); // test function for raycode

  rapidjson::Document toJSON();
};

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
