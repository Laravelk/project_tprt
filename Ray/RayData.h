#ifndef RAYDATA_H
#define RAYDATA_H

#include <memory>
#include <vector>

#include "../VelocityModel.hpp"
#include "Ray.hpp"

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

  void setTrajectory(const std::vector<double> &raw_trajectory) {
    std::vector<std::array<float, 3>> new_trajectory;
    int size = trajectory.size();
    trajectory.clear();
    trajectory.push_back(source.getLocation());
    for (int i = 1; i <= size - 2; i++) {
        float x = static_cast<float>(raw_trajectory[2 * (i - 1)]);
        float y = static_cast<float>(raw_trajectory[2 * (i - 1) + 1]);
      trajectory.push_back(
          { x, y,velocity_model->getLayer(ray_code[i].layerNumber)
               ->getTop()->getDepth({x, y})});
//      std::cerr << x << " " << y << " " << velocity_model->getLayer(ray_code[i].layerNumber)
//              ->getTop()->getDepth({x, y}) << std::endl;
    }
//    std::cerr << std::endl;
    trajectory.push_back(receiver.getLocation());
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
