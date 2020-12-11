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
class Ray {
private:
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
  std::vector<std::array<float, 3>> trajectory;

  /// function which optimize trajectory
  /// @return nothing
  void optimizeTrajectory();

  /// function do first optimization of trajectory
  /// @return nothing
  void computeSegmentsRay();

  void generateCode(const std::vector<std::array<int, 3>> ray_code);

  double calculate_part_time(const std::array<float, 3> source,
                             const std::array<float, 3> receiver,
                             int layer_number);
  double calculate_full_time(const std::vector<double> &x);
  double myfunc(const std::vector<double> &x, std::vector<double> &grad,
                void *data);

public:
  //  friend double myfunc(const std::vector<double> &x, std::vector<double>
  //  &grad,
  //                       void *data);

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

// double ray_tracing::Ray::myfunc(const std::vector<double> &x,
//                                std::vector<double> &grad, void *data) {
//  if (!grad.empty()) {
//    float xk_1 = trajectory[0][0]; // xk-1
//    float yk_1 = trajectory[0][1];
//    float zk_1 = trajectory[0][2];
//    unsigned long number_of_unknown = trajectory.size();

//    //    for (int i = 0; i < x.size(); i++) {
//    //      std::cerr << x[i] << " ";
//    //    }
//    //    std::cerr << std::endl;

//    for (long i = 0; i < number_of_unknown - 2; i++) {
//      float xk = trajectory[i + 1][0];
//      float yk = trajectory[i + 1][1];
//      float zk = trajectory[i + 1][2];

//      float xk_plus_1 = trajectory[i + 2][0];
//      float yk_plus_1 = trajectory[i + 2][1];
//      float zk_plus_1 = trajectory[i + 2][2];

//      std::array<double, 2> derive =
//          velocity_model->getLayer(ray_code[i].layerNumber)
//              ->getTop()
//              ->getGradientInPoint(xk, yk);

//      //      std::cerr << "-1 [" << xk_1 << ", " << yk_1 << ", " << zk_1 <<
//      "]"
//      //      << std::endl;
//      //      std::cerr << "0 [" << xk << ", " << yk << ", " << zk << "]" <<
//      //      std::endl; std::cerr << "1 [" << xk_plus_1 << ", " << yk_plus_1
//      <<
//      //      ", " << zk_plus_1
//      //                << "] " << std::endl;

//      float DzkDxk = (float)derive[0]; // dzk/dxk
//      float DzkDyk = (float)derive[1];

//      //      std::cerr << "DzkDxk: " << DzkDxk << std::endl;
//      //      std::cerr << "DzkDyk: " << DzkDyk << std::endl;

//      float Dvk_1Dxk = 0; // isotropic envoriment
//      float Dvk_1Dyk = 0; // isotropic envoriment

//      float DvkDxk_plus_1 = 0; // isotropic envoriment
//      float DvkDyk_plus_1 = 0; // isotropic envoriment

//      float vk =
//          (float)(velocity_model->getLayer(ray_code[i].layerNumber)->Vp); //
//          vk
//      //      std::cerr << "vk: " << vk << std::endl;
//      float vk_1 = 0;
//      int expression = i - 1;
//      if (expression < 0) {
//        vk_1 = (float)velocity_model->getLayer(0)->Vp;
//      } else {
//        vk_1 = (float)velocity_model->getLayer(ray_code[i - 1].layerNumber)
//                   ->Vp; // vk-1
//      }
//      //      std::cerr << "vk_1: " << vk_1 << std::endl;

//      float lk = sqrt((xk_plus_1 - xk) * (xk_plus_1 - xk) +
//                      (yk_plus_1 - yk) * (yk_plus_1 - yk) +
//                      (zk_plus_1 - zk) * (zk_plus_1 - zk));
//      //      std::cerr << "lk: " << lk << std::endl;

//      float lk_1 = sqrt((xk - xk_1) * (xk - xk_1) + (yk - yk_1) * (yk - yk_1)
//      +
//                        (zk - zk_1) * (zk - zk_1)); // lk-1
//      //      std::cerr << "lk_1: " << lk_1 << std::endl;

//      float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
//                    (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk);

//      //      std::cerr << "DtDxk: " << DtDxk << std::endl;

//      float DtDyk = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
//                    (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk);

//      //      std::cerr << "DtDyk: " << DtDyk << std::endl << std::endl;

//      grad[2 * i] = DtDxk;
//      grad[2 * i + 1] = DtDyk;

//      xk_1 = xk;
//      yk_1 = yk;
//      zk_1 = zk;
//    }
//  }
//  return calculate_full_time(x);
//}

} // namespace ray_tracing
#endif // TPRT_RAY_HPP
