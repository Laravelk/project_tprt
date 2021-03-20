#ifndef TPRT_LAYER_HPP
#define TPRT_LAYER_HPP

#include <memory>
#include <string>

#include "rapidjson/document.h"

#include "Ray/WaveType.h"
#include "Horizon/FlatHorizon.hpp"
#include "Horizon/GridHorizon.h"

namespace ray_tracing {
class Layer {
public:
  float Vp;
  float Vs;
  float density;

  std::unique_ptr<Horizon> top;
  std::string name;

  Layer(float Vp, float Vs, float density, std::unique_ptr<Horizon> itop,
        std::string name = "")
      : Vp(Vp), Vs(Vs), density(density), name(name) {
    top = std::move(itop);
  }

  Layer(std::unique_ptr<Horizon> _top, float _Vp, float _Vs, float _density)
      : Vp(_Vp), Vs(_Vs), density(_density), top(std::move(_top)) {}

  ~Layer() {}

  Layer &operator=(const Layer &rhs) {
    Vp = rhs.Vp;
    Vs = rhs.Vs;
    density = rhs.density;
    name = rhs.name;
    return *this;
  }

  float getVp() const { return Vp; }
  float getVs() const { return Vs; }
  float getDensity() const { return density; }
  Horizon *getTop() { return top.get(); }
  rapidjson::Document toJSON();
  static std::unique_ptr<Layer> fromJSON(const rapidjson::Value &doc);
};

} // namespace ray_tracing
#endif // TPRT_LAYER_HPP
