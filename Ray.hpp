#include <utility>

#ifndef TPRT_RAY_HPP
#define TPRT_RAY_HPP

#include "Layer.hpp"
#include "Receiver.hpp"
#include "Segment.hpp"
#include "Source.hpp"
#include "VelocityModel.hpp"

#include "cppoptlib/function.h"
#include "cppoptlib/solver/bfgs.h"
#include "cppoptlib/solver/conjugated_gradient_descent.h"
#include "cppoptlib/solver/gradient_descent.h"
#include "cppoptlib/solver/lbfgs.h"
#include "cppoptlib/solver/lbfgsb.h"
#include "cppoptlib/solver/newton_descent.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace cppoptlib;
using FunctionXd = cppoptlib::function::Function<double>;

namespace ray_tracing {

enum Direction { DOWN = -1, UP = 0 };
enum WaveType { SWAVE = 0, PWAVE = 1 };
/// class for cppoptlib Solver

/// source: object of type Source
/// receiver: object of type Receiver
/// velocity_model: object of type VelocityModel
/// ray_code: for each Segment of Ray is [+1 (down) or -1 (up), depth of
/// Layer, WaveType: 0 (WaveP) or 1 (WaveS)]
class Ray {
private:
  class Function : public FunctionXd {
  private:
    VelocityModel *model = nullptr;
    Ray &ray;

    /// source_location: source_cord
    /// receiver_location: receiver_cord
    /// iteration: i in ray_code
    double calculation_part_time(std::array<float, 3> source_location,
                                 std::array<float, 3> receiver_location,
                                 int layer_number) const {

      std::array<double, 3> vec{receiver_location[0] - source_location[0],
                                receiver_location[1] - source_location[1],
                                receiver_location[2] - source_location[2]};

      double norm_vec =
          sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
      auto layer = ray.velocity_model->getLayer(layer_number);
      double vp = static_cast<double>(layer->getVp());

      std::cerr << "norm_vec / vp " << norm_vec << " " << vp << std::endl;

      return norm_vec / vp;
    }

  public:
    using FunctionXd::hessian_t;
    using FunctionXd::vector_t;

    Function(Ray &_ray) : ray(_ray) {}

    scalar_t operator()(const vector_t &x) const override {
      double time = 0;
      int number_of_unknowns = ray.trajectory.size();
      for (int i = 1; i <= number_of_unknowns - 2; i++) {
        ray.trajectory[i] = {
            static_cast<float>(x[2 * (i - 1)]),
            static_cast<float>(x[2 * (i - 1) + 1]),
            ray.velocity_model->getLayer(ray.ray_code[i - 1].layerNumber)
                ->getTop()
                ->getDepth({static_cast<float>(x[2 * (i - 1)]),
                            static_cast<float>(x[2 * (i - 1) + 1])})};
      }

      std::array<float, 3> source_location = ray.source.getLocation();

      for (int i = 1; i < number_of_unknowns - 1; i++) {
        std::array<float, 3> receiver_location = ray.trajectory.at(i);
        double new_time =
            calculation_part_time(source_location, receiver_location,
                                  ray.ray_code.at(i - 1).layerNumber - 1);
        time += new_time;
        source_location = receiver_location;
      }

      int lastLayer = 0;
      if (Direction::DOWN ==
          ray.ray_code.at(ray.ray_code.size() - 2).direction) {
        lastLayer = ray.velocity_model->getLayersCount() - 1;
      } else {
        lastLayer = 0;
      }

      time += calculation_part_time(source_location, ray.receiver.getLocation(),
                                    lastLayer);

      std::cerr << std::endl;

      return time;
    }

    /*

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

    void Gradient(const vector_t &x, vector_t *grad) const override {
      float xk_1 = ray.trajectory[0][0]; // xk-1
      float yk_1 = ray.trajectory[0][1];
      float zk_1 = ray.trajectory[0][2];
      unsigned long number_of_unknown = ray.trajectory.size();

      for (int i = 0; i < x.size(); i++) {
        std::cerr << x[i] << " ";
      }
      std::cerr << std::endl;

      for (long i = 0; i < number_of_unknown - 2; i++) {
        float xk = ray.trajectory[i + 1][0];
        float yk = ray.trajectory[i + 1][1];
        float zk = ray.trajectory[i + 1][2];

        float xk_plus_1 = ray.trajectory[i + 2][0];
        float yk_plus_1 = ray.trajectory[i + 2][1];
        float zk_plus_1 = ray.trajectory[i + 2][2];

        std::array<double, 2> derive =
            ray.velocity_model->getLayer(ray.ray_code[i].layerNumber)
                ->getTop()
                ->getGradientInPoint(xk, yk);

        std::cerr << "-1 [" << xk_1 << ", " << yk_1 << ", " << zk_1 << "]"
                  << std::endl;
        std::cerr << "0 [" << xk << ", " << yk << ", " << zk << "]"
                  << std::endl;
        std::cerr << "1 [" << xk_plus_1 << ", " << yk_plus_1 << ", "
                  << zk_plus_1 << "] " << std::endl;

        float DzkDxk = (float)derive[0]; // dzk/dxk
        float DzkDyk = (float)derive[1];

        std::cerr << "DzkDxk: " << DzkDxk << std::endl;
        std::cerr << "DzkDyk: " << DzkDyk << std::endl;

        float Dvk_1Dxk = 0; // isotropic envoriment
        float Dvk_1Dyk = 0; // isotropic envoriment

        float DvkDxk_plus_1 = 0; // isotropic envoriment
        float DvkDyk_plus_1 = 0; // isotropic envoriment

        float vk =
            (float)(ray.velocity_model->getLayer(ray.ray_code[i].layerNumber)
                        ->Vp); // vk
        std::cerr << "vk: " << vk << std::endl;
        float vk_1 = 0;
        int expression = i - 1;
        if (expression < 0) {
          vk_1 = (float)ray.velocity_model->getLayer(0)->Vp;
        } else {
          vk_1 = (float)ray.velocity_model
                     ->getLayer(ray.ray_code[i - 1].layerNumber)
                     ->Vp; // vk-1
        }
        std::cerr << "vk_1: " << vk_1 << std::endl;

        float lk = sqrt((xk_plus_1 - xk) * (xk_plus_1 - xk) +
                        (yk_plus_1 - yk) * (yk_plus_1 - yk) +
                        (zk_plus_1 - zk) * (zk_plus_1 - zk));
        std::cerr << "lk: " << lk << std::endl;

        float lk_1 =
            sqrt((xk - xk_1) * (xk - xk_1) + (yk - yk_1) * (yk - yk_1) +
                 (zk - zk_1) * (zk - zk_1)); // lk-1
        std::cerr << "lk_1: " << lk_1 << std::endl;

        /*float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
                      lk_1 * Dvk_1Dxk / (vk_1 * vk_1) -
                      (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk)
           + lk * DvkDxk_plus_1 / (vk * vk);*/

        float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
                      (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk);

        std::cerr << "DtDxk: " << DtDxk << std::endl;

        /*float DtDxy = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
                      lk_1 * Dvk_1Dyk / (vk_1 * vk_1) -
                      (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk)
           + lk * DvkDyk_plus_1 / (vk * vk); */

        float DtDyk = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
                      (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk);

        std::cerr << "DtDyk: " << DtDyk << std::endl << std::endl;

        (*grad)[2 * i] = DtDxk;
        (*grad)[2 * i + 1] = DtDyk;

        xk_1 = xk;
        yk_1 = yk;
        zk_1 = zk;
      }
      std::cerr << "GRADIENT: ";
      for (int i = 0; i < (*grad).size(); i++) {
        std::cerr << (*grad)[i] << " ";
      }
      std::cerr << std::endl << std::endl;
    }
  };

  Source source;
  Receiver receiver;
  VelocityModel *velocity_model;

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
  }

  [[deprecated]] Ray(Source source, Receiver receiver, VelocityModel *_model)
      : source(std::move(source)), receiver(receiver), velocity_model(_model),
        timeP(INFINITY), timeS(INFINITY), amplitudeP(1), amplitudeS(1) {}

  [[deprecated]] void computePath();
  void computePathWithRayCode(); // test function for raycode

  rapidjson::Document toJSON();
};

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
