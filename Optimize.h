#ifndef OPTIMIZE_H
#define OPTIMIZE_H

#endif // OPTIMIZE_H

#include "Ray/Ray.hpp"
#include "Ray/RayData.h"

namespace ray_tracing {
class Optimize {
public:
  Optimize() {}

  static double myfunc(const std::vector<double> &, std::vector<double> &grad,
                       void *my_func_data) {
    auto *ray_data = static_cast<ray_tracing::RayData *>(my_func_data);

    auto trajectory = ray_data->trajectory;
    auto ray_code = ray_data->ray_code;
    auto horizons = ray_data->horizons;
    auto vp = ray_data->vp;

    if (!grad.empty()) {
      float xk_1 = trajectory[0][0]; // xk-1
      float yk_1 = trajectory[0][1];
      float zk_1 = trajectory[0][2];
      unsigned long number_of_unknown = trajectory.size();

      for (unsigned long i = 0; i < number_of_unknown - 2; i++) {
        float xk = trajectory[i + 1][0];
        float yk = trajectory[i + 1][1];
        float zk = trajectory[i + 1][2];

        float xk_plus_1 = trajectory[i + 2][0];
        float yk_plus_1 = trajectory[i + 2][1];
        float zk_plus_1 = trajectory[i + 2][2];

        std::array<double, 2> derive =
            horizons[ray_code[i].layerNumber]->getGradientInPoint(xk, yk);

        float DzkDxk = (float)derive[0]; // dzk/dxk
        float DzkDyk = (float)derive[1];

        //        std::cerr << "DzkDxk: " << DzkDxk << std::endl;
        //        std::cerr << "DzkDyk: " << DzkDyk << std::endl;

        //        float Dvk_1Dxk = 0; // isotropic envoriment
        //        float Dvk_1Dyk = 0; // isotropic envoriment

        //        float DvkDxk_plus_1 = 0; // isotropic envoriment
        //        float DvkDyk_plus_1 = 0; // isotropic envoriment

        float vk = vp[ray_code[i].layerNumber];

        //        std::cerr << "vk: " << vk << std::endl;
        float vk_1 = 0;
        int expression = i - 1;
        if (expression < 0) {
          vk_1 = vp[0];
        } else {
          vk_1 = vp[ray_code[i - 1].layerNumber]; // vk_1
        }
        //        std::cerr << "vk_1: " << vk_1 << std::endl;

        float lk = sqrt((xk_plus_1 - xk) * (xk_plus_1 - xk) +
                        (yk_plus_1 - yk) * (yk_plus_1 - yk) +
                        (zk_plus_1 - zk) * (zk_plus_1 - zk));
        //        std::cerr << "lk: " << lk << std::endl;

        float lk_1 =
            sqrt((xk - xk_1) * (xk - xk_1) + (yk - yk_1) * (yk - yk_1) +
                 (zk - zk_1) * (zk - zk_1)); // lk-1
        //        std::cerr << "lk_1: " << lk_1 << std::endl;

        float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
                      (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk);

        //        std::cerr << "DtDxk: " << DtDxk << std::endl;

        float DtDyk = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
                      (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk);

        //        std::cerr << "DtDyk: " << DtDyk << std::endl << std::endl;

        grad[2 * i] = DtDxk;
        grad[2 * i + 1] = DtDyk;

        xk_1 = xk;
        yk_1 = yk;
        zk_1 = zk;
      }
    }
    return calculate_full_time(ray_data);
  }

private:
  static double calculate_part_time(const std::array<float, 3> source,
                                    const std::array<float, 3> receiver,
                                    int layer_number,
                                    ray_tracing::RayData *ray_data) {
    auto velocity_model = ray_data->velocity_model;

    std::array<double, 3> vec{receiver[0] - source[0], receiver[1] - source[1],
                              receiver[2] - source[2]};

    double norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    auto layer = velocity_model->getLayer(layer_number);
    double vp = static_cast<double>(layer->getVp());

    return norm_vec / vp;
  }

  static double calculate_full_time(ray_tracing::RayData *ray_data) {
    double time = 0;
    auto trajectory = ray_data->trajectory;
    auto velocity_model = ray_data->velocity_model;
    auto ray_code = ray_data->ray_code;
    auto vp_array = ray_data->vp;
    int number_of_unknowns = trajectory.size();

    for (int i = 1; i < number_of_unknowns - 1; i++) {
      std::array<double, 3> vec{trajectory[i][0] - trajectory[i - 1][0],
                                trajectory[i][1] - trajectory[i - 1][1],
                                trajectory[i][2] - trajectory[i - 1][2]};

      double norm_vec =
          sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
      float vp = vp_array[ray_code.at(i - 1).layerNumber - 1];

      time += norm_vec / vp;
    }

    std::array<double, 3> vec{trajectory[number_of_unknowns - 1][0] -
                                  trajectory[number_of_unknowns - 2][0],
                              trajectory[number_of_unknowns - 1][1] -
                                  trajectory[number_of_unknowns - 2][1],
                              trajectory[number_of_unknowns - 1][2] -
                                  trajectory[number_of_unknowns - 2][2]};

    double norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    double vp = vp_array[velocity_model->getLayersCount() - 1];

    time += norm_vec / vp;

    return time;
  }
};
} // namespace ray_tracing
