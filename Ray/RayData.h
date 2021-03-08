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
