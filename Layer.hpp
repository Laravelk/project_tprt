#ifndef TPRT_LAYER_HPP
#define TPRT_LAYER_HPP

#include "Horizon/FlatHorizon.hpp"
#include "Horizon/GridHorizon.h"
#include "rapidjson/document.h"
#include <memory>
#include <string>

/*
 * Среда
 * */
namespace ray_tracing {
class Layer {
public:
  float Vp;
  float Vs;

  std::unique_ptr<Horizon> top;
  std::string name;

  Layer(float Vp, float Vs, std::unique_ptr<Horizon> itop,
        std::string name = "")
      : Vp(Vp), Vs(Vs), name(name) {
    top = std::move(itop);
  }

  Layer(const Layer &rhs) : Vp(rhs.Vp), Vs(rhs.Vs), name(rhs.name) {
    std::cerr << "HHHH";
    // this->top = rhs.top->clone();
  }

  ~Layer() {}

  Layer &operator=(const Layer &rhs) {
    Vp = rhs.Vp;
    Vs = rhs.Vs;
    name = rhs.name;
    return *this;
  }

  float getVp() const { return Vp; }
  float getVs() const { return Vs; }
  Horizon *getTop() { return top.get(); }
  rapidjson::Document toJSON();
  static Layer fromJSON(const rapidjson::Value &doc);
};

} // namespace ray_tracing
#endif // TPRT_LAYER_HPP
