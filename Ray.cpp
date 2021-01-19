#include "Ray.hpp"
#include "Segment.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <ctime>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>

#include "Optimize.h"

#include <nlopt.h>
#include <nlopt.hpp>

// check module -- проверяет, что слои все хорошо

typedef unsigned long ulong;

/// this namespace contains classes which worked with ray_code and trajectory
namespace ray_tracing {
void Ray::optimizeTrajectory() {
  std::vector<double> vector;

  for (int i = 1; i < trajectory.size() - 1; i++) {
    vector.push_back(trajectory.at(i)[0]);
    vector.push_back(trajectory.at(i)[1]);
  }

  /* Примерная итоговая траектория
[ 0, 0, 0]
[ 24.4568, 26.1691, 50]
[ 59.8238, 63.9923, 103.863]
[ 168.736, 178.158, 203.104]
[ 314.162, 325.188, 300]
[ 777, 777, 400]
Time 0.379
  */
  // LD_SLSQP – 0.379634 Python + За 200 итераций результат – 0.389

  // LD_MMA - 0.385 Python +- За 70-80 итераций
  /*
    [ 0, 0, 0]
    [ 53.254, 53.5549, 50]
    [ 69.4059, 69.3922, 104.324]
    [ 297.961, 297.914, 205.466]
    [ 479.379, 479.358, 300]
    [ 777, 777, 400]
  */

  // LD_CCSAQ - 0.386 За 100-200 итераций
  /*
    [ 0, 0, 0]
    [ 51.2585, 51.2338, 50]
    [ 114.054, 114.572, 107.025]
    [ 333.097, 332.144, 206.11]
    [ 506.851, 507.61, 300]
    [ 777, 777, 400]
  */

  // LD_AUGLAG - 0.388 150 итераций
  /*
    [ 0, 0, 0]
    [ 53.254, 53.5549, 50]
    [ 69.4059, 69.3922, 104.324]
    [ 297.961, 297.914, 205.466]
    [ 479.379, 479.358, 300]
    [ 777, 777, 400]
  */

  // LD_LBFGS terminating with uncaught exception of type std::runtime_error:
  // nlopt failure
  // LD_TNEWTON_PRECOND_RESTART аналогично
  // LD_TNEWTON аналогично
  // LD_VAR1 аналогично

  // LN_COBYLA – OutOfMemory Python (2000) -  Эффективность сомнительная
  // LN_BOBYQA OutOfMemory при 500 итерация. Выглядит более эффективным
  // LN_NEWUOA Рассходится на определенной итерации > 1200
  // LN_PRAXIS 2000 итераций. Плохо сходится
  // LN_NELDERMEAD > 5000 итераций. Плохо сходится
  // LN_SBPLX Не меняет значение траектории вовсе

  nlopt::opt opt(nlopt::LD_SLSQP, vector.size());
  std::vector<double> lb(vector.size());
  for (auto v : lb) {
    v = -1000;
  }
  double minf = 0;
  opt.set_lower_bounds(lb);
  opt.set_min_objective(Optimize::myfunc, this);
  opt.set_xtol_abs(1e-3);
  opt.set_maxeval(5);
  nlopt::result result = opt.optimize(vector, minf);
  //  std::cout << "The result is" << std::endl;
  //  std::cout << result << std::endl;
  //  std::cout << "Minimal function value " << minf << std::endl;
  //  std::cout << "search_time: " << search_time / CLOCKS_PER_SEC << std::endl;
  //  for (auto tr : trajectory) {
  //    std::cerr << "[ " << tr.at(0) << ", " << tr.at(1) << ", " << tr.at(2)
  //              << "] " << std::endl;
  //  }
}

/// compute ray in layer and create segments
void Ray::computeSegmentsRay() {
  auto source_location = current_source.getLocation();
  auto receiver_location = current_receiver.getLocation();

  float diff_x = receiver_location[0] - source_location[0];
  float diff_y = receiver_location[1] - source_location[1];
  ulong trajectory_part_count = ray_code.size();
  float step_x = diff_x / trajectory_part_count;
  float step_y = diff_y / trajectory_part_count;
  float x = 0, y = 0, z = 0;
  x = source_location[0];
  y = source_location[1];

  trajectory.push_back({x, y, source_location[2]});
  for (ulong i = 0; i < trajectory_part_count - 1; i++) {
    x += step_x;
    y += step_y;
    Horizon *hor =
        velocity_model->getLayer(ray_code.at(i).layerNumber)->getTop();
    z = hor->getDepth({x, y});
    trajectory.push_back({x, y, z});
  }
  trajectory.push_back(
      {receiver_location[0], receiver_location[1], receiver_location[2]});
  //  for (auto tr : trajectory) {
  //    std::cerr << tr[0] << " " << tr[1] << " " << tr[2] << std::endl;
  //  }
}

void Ray::generateCode(const std::vector<std::array<int, 3>> rayCode) {
  for (auto ray_element : rayCode) {
    Direction direction;
    WaveType type;
    if (Direction::DOWN == ray_element[1]) {
      direction = Direction::DOWN;
    } else {
      direction = Direction::UP;
    }
    if (WaveType::SWAVE == ray_element[2]) {
      type = WaveType::SWAVE;
    } else {
      type = WaveType::PWAVE;
    }
    Code code(ray_element[0], direction, type);
    ray_code.push_back(code);
  }
}

void Ray::computePath() {}

void Ray::computePathWithRayCode() {
  double start_time = clock(); // начальное время
  double end_time = clock();   // конечное время

  //  computeSegmentsRay();
  //  current_receiver = receivers[0];
  //  current_source = sources[0];
  std::cerr << "FFF";
  for (long i = 0; i < receivers.size(); i++) {
    start_time = clock();
    current_receiver = receivers[i];
    current_source = sources[i];
    trajectory.clear();
    computeSegmentsRay();
    optimizeTrajectory();
    end_time = clock();
    double search_time = end_time - start_time; // искомое время
    std::cerr << search_time / CLOCKS_PER_SEC << std::endl;
  }
}

rapidjson::Document Ray::toJSON() {
  rapidjson::Document doc;
  /*rapidjson::Value json_val;
  rapidjson::Value tmp_json_val;
  doc.SetObject();

  auto &allocator = doc.GetAllocator();

  json_val.CopyFrom(source.toJSON(), allocator);
  doc.AddMember("Source", json_val, allocator);

  json_val.CopyFrom(receiver.toJSON(), allocator);
  doc.AddMember("Receiver", json_val, allocator);

  json_val.SetArray();
  auto trajectory = getTrajectoryP();
  for (const auto &tr_seg : trajectory) {
    tmp_json_val.SetArray();
    tmp_json_val.PushBack(tr_seg[0], allocator)
        .PushBack(tr_seg[1], allocator)
        .PushBack(tr_seg[2], allocator);
    json_val.PushBack(tmp_json_val, allocator);
  }
  doc.AddMember("TrajectoryP", json_val, allocator);

  json_val.SetArray();
  trajectory = getTrajectoryS();
  for (const auto &tr_seg : trajectory) {
    tmp_json_val.SetArray();
    tmp_json_val.PushBack(tr_seg[0], allocator)
        .PushBack(tr_seg[1], allocator)
        .PushBack(tr_seg[2], allocator);
    json_val.PushBack(tmp_json_val, allocator);
  }
  doc.AddMember("TrajectoryS", json_val, allocator);

  json_val.SetFloat(amplitudeP);
  doc.AddMember("AmplitudeP", json_val, allocator);

  json_val.SetFloat(timeP);
  doc.AddMember("TimeP", json_val, allocator);

  json_val.SetFloat(amplitudeS);
  doc.AddMember("AmplitudeS", json_val, allocator);

  json_val.SetFloat(timeS);
  doc.AddMember("TimeS", json_val, allocator);

  json_val.SetString("NONE");
  doc.AddMember("Record", json_val, allocator);
  */
  return doc;
} // namespace ray_tracing

} // namespace ray_tracing
