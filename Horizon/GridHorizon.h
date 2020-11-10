#ifndef TPRT_GRIDHORIZON_H
#define TPRT_GRIDHORIZON_H

// Created by Иван Морозов on 2020-06-17.

#include "Horizon.h"
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <array>
#include <float.h>
#include <string>

namespace ray_tracing {
class GridHorizon : public Horizon {
private:
  _2D::BicubicInterpolator<float> interpolator;
  std::vector<std::tuple<float, float, float>> points;

  std::array<float, 2> right_top = {FLT_MIN, FLT_MIN};
  std::array<float, 2> left_top = {FLT_MAX, FLT_MIN};
  std::array<float, 2> right_bottom = {FLT_MIN, FLT_MAX};
  std::array<float, 2> left_bottom = {FLT_MAX, FLT_MAX};

  float gradient_step = 0.0f;

  bool checkGrid(std::vector<float> &, std::vector<float> &,
                 std::vector<float> &);

  void find_corner();

  /*
   * x: x cord
   * y: y cord
   * @return gradient in (x,y)
   * */
  std::vector<float> calculateGradientInPoint(float x, float y) const;
  std::vector<float> normalAtPoint(float x, float y, float z) const;
  std::vector<float> minimize(const std::array<float, 3> &x0,
                              const std::array<float, 3> &x1,
                              const std::array<float, 3> &vector) const;

  constexpr static double EPS = 0.000001;

public:
  /*
   * anchor: first point of horizon
   * normal: vector of normal vectors for all points
   * name: name of horizon
   * points: all points after interpolation
   */
  GridHorizon(std::string, std::vector<std::tuple<float, float, float>>);

  /* copy constructor */
  GridHorizon(GridHorizon &hor)
      : interpolator(hor.getInterpolator()), points(hor.getPoints()),
        gradient_step(hor.gradient_step) {
    name = hor.getName();
  }

  /* destructor */
  ~GridHorizon() override {}

  // static method
  // points: vector of points
  // step: grid step
  // @return _2D::BicubicInterpolator
  //        static _2D::BicubicInterpolator<float> st_interpolation
  //                (const std::vector<std::tuple<float, float, float>>&
  //                points);

  // non-static method
  // points: vector of points
  // step: grid step
  // @return true if interpolation success else return false
  bool
  interpolation(const std::vector<std::tuple<float, float, float>> &points);

  virtual rapidjson::Document toJSON() override;

  float getDepth(std::array<float, 2> cord) const override;
  float getDepth(float x, float y) const;

  /* calculate intersect
   * x0: first point (source)
   * x1: second point (receiver)
   * @return interest point */
  std::vector<float>
  calcIntersect(const std::array<float, 3> &x0,
                const std::array<float, 3> &x1) const override;

  std::array<double, 2> getGradientInPoint(double x, double y) const override;
  std::array<double, 2>
  getGradientInPoint(std::array<double, 2> cord) const override;

  /* get GridHorizon from JSON file */
  static std::unique_ptr<GridHorizon> fromJSON(const rapidjson::Value &doc);

  /* @return z value for x, y */
  double operator()(float x, float y) const;

  virtual Horizon *clone() override;

  std::string getName() const { return name; }
  _2D::BicubicInterpolator<float> getInterpolator() const;
  void setInterpolator(const _2D::BicubicInterpolator<float> &value);
  std::vector<std::tuple<float, float, float>> getPoints() const;
  void setPoints(const std::vector<std::tuple<float, float, float>> &value);
};
} // namespace ray_tracing

#endif // TPRT_GRIDHORIZON_H
