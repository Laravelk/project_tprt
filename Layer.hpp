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

  Layer(float Vp, float Vs, std::unique_ptr<Horizon> top, std::string name = "")
      : Vp(Vp), Vs(Vs), top(std::move(top)), name(name) {}

  Layer(const Layer &rhs)
      : Vp(rhs.Vp), Vs(rhs.Vs), top(std::move(rhs.top.get())), name(rhs.name) {}

  //  Layer &operator=(const Layer &rhs) {
  //    Vp = rhs.Vp;
  //    Vs = rhs.Vs;
  //    std::top = std::move(rhs.top);
  //    name = rhs.name;
  //    return *this;
  //  }

  float getVp() const { return Vp; }

  float getVs() const { return Vs; }

  std::unique_ptr<Horizon> &getTop() { return top; }

  rapidjson::Document toJSON();

  static Layer fromJSON(const rapidjson::Value &doc);
};

} // namespace ray_tracing
#endif // TPRT_LAYER_HPP
