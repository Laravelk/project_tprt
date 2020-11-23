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
#include "cppoptlib/solver/lbfgsbsolver.h"
#include "cppoptlib/solver/lbfgssolver.h"
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

      std::array<double, 3> vec{receiver_location[0] - source_location[0],
                                receiver_location[1] - source_location[1],
                                receiver_location[2] - source_location[2]};

      double norm_vec =
          sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
      auto layer =
          ray.velocity_model->getLayer(ray.ray_code.at(iteration).layerNumber);

      return norm_vec / static_cast<double>(layer->getVp());
    }

  public:
    using typename cppoptlib::Problem<double>::Scalar;
    using typename cppoptlib::Problem<double>::TVector;
    MyProblem(Ray &_ray) : ray(_ray) {}

    double value(const TVector &x) {
      double time = 0;
      std::cerr << "Ray::value::start" << std::endl;
      int number_of_unknowns = ray.ray_code.size();
      for (int i = 1; i < number_of_unknowns - 1; i++) {
        ray.trajectory[i] = {
            static_cast<float>(x[2 * (i - 1)]),
            static_cast<float>(x[2 * (i - 1) + 1]),
            ray.velocity_model->getLayer(ray.ray_code[i].layerNumber)
                ->getTop()
                ->getDepth({static_cast<float>(x[2 * (i - 1)]),
                            static_cast<float>(x[2 * (i - 1) + 1])})};
      }

      std::array<float, 3> source_location = ray.source.getLocation();

      for (int i = 1; i < ray.trajectory.size() - 1; i++) {
        std::array<float, 3> receiver_location = ray.trajectory.at(i);
        time += calculation_part_time(source_location, receiver_location, i);
        source_location = receiver_location;
      }

      time += calculation_part_time(
          source_location, ray.receiver.getLocation(),
          ray.ray_code.at(ray.ray_code.size() - 1).layerNumber);

      std::cerr << "return time " << time
                << "\n"; // TODO: delete or #ifdef debug
      return time;
    }

    /*
 Computes derivatives along the Ray with respect to [x,y] coordiantes of the ray
on each horizon (interface). Equation (I.R. Obolentseva, V.Yu. Grechka, Ray
method in anisotropic medium, 1989) k - number of a point [x_k, y_k, z_k], v_k,
l_k - velocity and distance between k+1 and k points

    dt/dx_k = (x_k - x_k-1 + (z_k - z_k-1) * dz_k/dx_k) / (l_k-1 * v_k-1) -
l_k-1 * dv_k-1/dx_k / v_k-1**2 - (x_k+1 - x_k + (z_k+1 - z_k) * dz_k/dx_k) /
(l_k * v_k)
                + l_k * dv_k/dx_k+1 / v_k**2;
    dt/dy_k = (y_k - y_k-1 + (z_k - z_k-1) * dz_k/dy_k)
                        / (l_k-1 * v_k-1) - l_k-1 * dv_k-1/dy_k / v_k-1**2 -
(y_k+1 - y_k + (z_k+1 - z_k) * dz_k/dy_k) / (l_k * v_k) + l_k * dv_k/dy_k+1 /
v_k**2

                                                    Return:
        dtdxy: numpy array (N-2, 2)
                    N is number of points in trajectory + source and receiver,
each is [dt/dxi, dt/dyi].

             """
                 */

    void gradient(const TVector &x, TVector &grad) override {
      std::cerr << "Ray::gradient::start" << std::endl;

      float xk_1 = ray.trajectory[0][0]; // xk-1
      float yk_1 = ray.trajectory[0][1];
      float zk_1 = ray.trajectory[0][2];
      unsigned long number_of_unknown = ray.trajectory.size();

      for (long i = 0; i <= number_of_unknown - 2; i++) {
        float xk = ray.trajectory[i + 1][0];
        float yk = ray.trajectory[i + 1][1];
        float zk = ray.trajectory[i + 1][2];

        float xk_plus_1 = ray.trajectory[i + 2][0];
        float yk_plus_1 = ray.trajectory[i + 2][1];
        float zk_plus_1 = ray.trajectory[i + 2][2];

        std::array<double, 2> gradient =
            ray.velocity_model->getLayer(ray.ray_code[i].layerNumber)
                ->getTop()
                ->getGradientInPoint(xk, yk);

        float DzkDxk = (float)gradient[0]; // dzk/dxk
        float DzkDyk = (float)gradient[1];

        float Dvk_1Dxk = 0; // isotropic envoriment
        float Dvk_1Dyk = 0; // isotropic envoriment

        float DvkDxk_plus_1 = 0; // isotropic envoriment
        float DvkDyk_plus_1 = 0; // isotropic envoriment

        float vk =
            (float)(ray.velocity_model->getLayer(ray.ray_code[i].layerNumber)
                        ->Vp); // vk
        float vk_1 = 0;
        int expression = i - 1;
        if (expression < 0) {
          vk_1 = 1000;
        } else {
          vk_1 = (float)ray.velocity_model
                     ->getLayer(ray.ray_code[i - 1].layerNumber)
                     ->Vp; // vk-1
        }

        float lk = sqrt((xk_plus_1 - xk) * (xk_plus_1 - xk) +
                        (yk_plus_1 - yk) * (yk_plus_1 - yk) +
                        (zk_plus_1 - zk) * (zk_plus_1 - zk));

        float lk_1 =
            sqrt((xk - xk_1) * (xk - xk_1) + (yk - yk_1) * (yk - yk_1) +
                 (zk - zk_1) * (zk - zk_1)); // lk-1

        /*float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
                      lk_1 * Dvk_1Dxk / (vk_1 * vk_1) -
                      (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk) +
                      lk * DvkDxk_plus_1 / (vk * vk);*/

        float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
                      (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk);

        /*float DtDxy = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
                      lk_1 * Dvk_1Dyk / (vk_1 * vk_1) -
                      (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk) +
                      lk * DvkDyk_plus_1 / (vk * vk); */

        float DtDxy = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
                      (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk);

        grad[2 * i] = DtDxk;
        grad[2 * i + 1] = DtDxy;

        xk_1 = xk;
        yk_1 = yk;
        zk_1 = zk;
      }
      std::cerr << "GRAD: ";
      std::cerr << grad << std::endl;
      std::cerr << std::endl;
    }
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
