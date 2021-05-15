#ifndef TPRT_GRIDHORIZON_H
#define TPRT_GRIDHORIZON_H

// Created by Иван Морозов on 2020-06-17.

#include "Horizon.h"
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <array>
#include <Eigen/Dense>
#include <string>

namespace ray_tracing {
class GridHorizon : public Horizon {
private:
  _2D::BicubicInterpolator<float> interpolator;
  std::vector<std::tuple<float, float, float>> points;

  float gradient_step = 0.0f;

  bool checkGrid(std::vector<float> &, std::vector<float> &,
                 std::vector<float> &);

  /*
   * x: x cord
   * y: y cord
   * @return gradient in (x,y)
   * */
  std::array<float, 2> calculateGradientInPoint(float x, float y) const;

  constexpr static double EPS = 0.000001;

public:
  std::vector<std::array<float, 2>> region;

  GridHorizon(std::string, std::vector<std::tuple<float, float, float>>,
              std::vector<std::array<float, 2>>);

  /* copy constructor */
  GridHorizon(GridHorizon &hor)
      : interpolator(hor.getInterpolator()), points(hor.getPoints()),
        gradient_step(hor.gradient_step), region(hor.region) {
    name = hor.getName();
  }

  /* destructor */
  ~GridHorizon() override {}

  bool
  interpolation(const std::vector<std::tuple<float, float, float>> &points);

  virtual rapidjson::Document toJSON() override;

  float getDepth(std::array<float, 2> cord) const override;
  float getDepth(float x, float y) const;

  std::array<float, 2> getGradientInPoint(double x, double y) const override;
  std::array<float, 2>
  getGradientInPoint(std::array<double, 2> cord) const override;

  /* get GridHorizon from JSON file */
  static std::unique_ptr<GridHorizon> fromJSON(const rapidjson::Value &doc);

  /* @return z value for x, y */
  double operator()(float x, float y) const;

  std::string getName() const { return name; }

  _2D::BicubicInterpolator<float> getInterpolator() const;
  void setInterpolator(const _2D::BicubicInterpolator<float> &value);

  std::vector<std::tuple<float, float, float>> getPoints() const;

  std::vector<float> getNormal(std::array<float, 2> cord) const;

  Eigen::Matrix2f getHessian(const float x, const float y) const;

};
} // namespace ray_tracing

#endif // TPRT_GRIDHORIZON_H
