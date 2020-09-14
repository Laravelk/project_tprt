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

  Horizon *top;
  std::string name;

  Layer(float Vp, float Vs, Horizon *itop, std::string name = "")
      : Vp(Vp), Vs(Vs), top(itop), name(name) {}

  Layer(const Layer &rhs) : Vp(rhs.Vp), Vs(rhs.Vs), name(rhs.name) {
    this->top = rhs.top->clone();
  }

  ~Layer() { delete top; }

  Layer &operator=(const Layer &rhs) {
    Vp = rhs.Vp;
    Vs = rhs.Vs;
    this->top = top->clone();
    name = rhs.name;
    return *this;
  }

  float getVp() const { return Vp; }

  float getVs() const { return Vs; }

  Horizon *getTop() { return top; }

  rapidjson::Document toJSON();

  static Layer fromJSON(const rapidjson::Value &doc);
};

} // namespace ray_tracing
#endif // TPRT_LAYER_HPP
