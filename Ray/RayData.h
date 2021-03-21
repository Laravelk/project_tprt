#ifndef RAYDATA_H
#define RAYDATA_H

#include <memory>
#include <vector>

#include "Ray.hpp"
#include "../VelocityModel.hpp"

namespace ray_tracing {
struct RayData {
public:
  RayData(Ray *ray) : source(ray->getSource()), receiver(ray->getReceiver()) {
    trajectory = ray->getTrajectory();
    ray_code = ray->getRayCodeVector();
    velocity_model = ray->getModel();
    layers = velocity_model->getLayers();

    for (auto layer : layers) {
      horizons.push_back(layer->getTop());
      vp.push_back(layer->getVp());
    }
  }

    void setTrajectory(std::vector<double> raw_trajectory) {
        std::vector<std::array<float, 3>> new_trajectory;
        int size = trajectory.size();
        trajectory.clear();
        trajectory.push_back(source.getLocation());
        for (int i = 1; i <= size - 2; i++) {
            trajectory.push_back(
                    {static_cast<float>(raw_trajectory[2 * (i - 1)]),
                     static_cast<float>(raw_trajectory[2 * (i - 1) + 1]),
                     velocity_model->getLayer(ray_code[i].layerNumber)
                             ->getTop()
                             ->getDepth(
                                     {static_cast<float>(raw_trajectory[2 * (i - 1)]),
                                      static_cast<float>(raw_trajectory[2 * (i - 1) + 1])})});
        }
        trajectory.push_back(receiver.getLocation());

            std::cerr << "trajectory" << std::endl;
            for (auto part : trajectory) {
              std::cerr << part[0] << " " << part[1] << " " << part[2] <<
              std::endl;
            }
            std::cerr << std::endl;
    }


    std::vector<std::array<float, 3>> trajectory;
  VelocityModel *velocity_model;
  std::vector<Code> ray_code;
  std::vector<Layer *> layers;
  std::vector<float> vp;
  std::vector<Horizon *> horizons;
  Source source;
  Receiver receiver;
};
} // namespace ray_tracing

#endif // RAYDATA_H
