#ifndef OPTIMIZE_H
#define OPTIMIZE_H

#endif // OPTIMIZE_H

#include "Ray.hpp"

namespace ray_tracing {
class Optimize {
public:
  static double myfunc(const std::vector<double> &x, std::vector<double> &grad,
                       void *my_func_data) {
    Ray *ray = static_cast<ray_tracing::Ray *>(my_func_data);

    auto trajectory = ray->getTrajectory();
    auto velocity_model = ray->getModel();
    auto ray_code = ray->getRayCodeVector();

    if (!grad.empty()) {
      float xk_1 = trajectory[0][0]; // xk-1
      float yk_1 = trajectory[0][1];
      float zk_1 = trajectory[0][2];
      unsigned long number_of_unknown = trajectory.size();

      for (long i = 0; i < number_of_unknown - 2; i++) {
        float xk = trajectory[i + 1][0];
        float yk = trajectory[i + 1][1];
        float zk = trajectory[i + 1][2];

        float xk_plus_1 = trajectory[i + 2][0];
        float yk_plus_1 = trajectory[i + 2][1];
        float zk_plus_1 = trajectory[i + 2][2];

        std::array<double, 2> derive =
            velocity_model->getLayer(ray_code[i].layerNumber)
                ->getTop()
                ->getGradientInPoint(xk, yk);

        float DzkDxk = (float)derive[0]; // dzk/dxk
        float DzkDyk = (float)derive[1];

        //        std::cerr << "DzkDxk: " << DzkDxk << std::endl;
        //        std::cerr << "DzkDyk: " << DzkDyk << std::endl;

        float Dvk_1Dxk = 0; // isotropic envoriment
        float Dvk_1Dyk = 0; // isotropic envoriment

        float DvkDxk_plus_1 = 0; // isotropic envoriment
        float DvkDyk_plus_1 = 0; // isotropic envoriment

        float vk = (float)(velocity_model->getLayer(ray_code[i].layerNumber)
                               ->Vp); // vk
        //        std::cerr << "vk: " << vk << std::endl;
        float vk_1 = 0;
        int expression = i - 1;
        if (expression < 0) {
          vk_1 = (float)velocity_model->getLayer(0)->Vp;
        } else {
          vk_1 = (float)velocity_model->getLayer(ray_code[i - 1].layerNumber)
                     ->Vp; // vk-1
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

        //        std::cerr << "grad: " << std::endl;
        //        for (int i = 0; i < grad.size(); i++) {
        //          std::cerr << grad[i] << std::endl;
        //        }
        //        std::cerr << std::endl;

        xk_1 = xk;
        yk_1 = yk;
        zk_1 = zk;
      }
    }
    return calculate_full_time(x, ray);
  }

private:
  static double calculate_part_time(const std::array<float, 3> source,
                                    const std::array<float, 3> receiver,
                                    int layer_number, ray_tracing::Ray *ray) {
    auto velocity_model = ray->getModel();

    std::array<double, 3> vec{receiver[0] - source[0], receiver[1] - source[1],
                              receiver[2] - source[2]};

    double norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    auto layer = velocity_model->getLayer(layer_number);
    double vp = static_cast<double>(layer->getVp());

    //    std::cerr << "norm_vec / vp " << norm_vec << " " << vp << " "
    //              << norm_vec / vp << std::endl
    //              << std::endl;

    return norm_vec / vp;
  }

  static double calculate_full_time(const std::vector<double> &x,
                                    ray_tracing::Ray *ray) {
    double time = 0;
    auto trajectory = ray->getTrajectory();
    auto velocity_model = ray->getModel();
    auto ray_code = ray->getRayCodeVector();
    Source source = ray->getSource();
    Receiver receiver = ray->getReceiver();

    int number_of_unknowns = trajectory.size();

    ray->setTrajectory(x);

    std::array<float, 3> source_location = source.getLocation();

    for (int i = 1; i < number_of_unknowns - 1; i++) {
      std::array<float, 3> receiver_location = trajectory.at(i);
      time += calculate_part_time(source_location, receiver_location,
                                  ray_code.at(i - 1).layerNumber - 1, ray);
      source_location = receiver_location;
    }

    int lastLayer = 0;
    if (ray_tracing::Direction::DOWN ==
        ray_code.at(ray_code.size() - 2).direction) {
      lastLayer = velocity_model->getLayersCount() - 1;
    } else {
      lastLayer = 0;
    }

    time += calculate_part_time(source_location, receiver.getLocation(),
                                lastLayer, ray);
    
    return time;
  }
};
} // namespace ray_tracing
