#include <utility>
#include <utility>
#include "Ray.hpp"
#include "Segment.hpp"
#include <vnl/vnl_cost_function.h>
#include <vnl/algo/vnl_amoeba.h>
#include "Eigen/Dense"
#include <unsupported/Eigen/NumericalDiff>


namespace ray_tracing {
    void Ray::computeSegments() {
        std::vector<std::tuple<float, std::array<float, 3>, Layer>> intersections;

        auto source_location = source.getLocation();
        auto receiver_location = receiver.getLocation();

        for (const Layer &l: velocity_model.getLayers()) {

            auto intersect = l.getTop().calcIntersect(source_location, receiver_location);

            if (!isnan(intersect[0])) {
                auto dist = sqrt(
                        pow(intersect[0] - source_location[0], 2.0f) + pow(intersect[1] - source_location[1], 2.0f) +
                        pow(intersect[2] - source_location[2], 2.0f));
                intersections.emplace_back(dist, intersect, l);
            }
        }

        std::sort(intersections.begin(), intersections.end(),
                  [](const std::tuple<float, std::array<float, 3>, Layer> &a,
                     const std::tuple<float, std::array<float, 3>, Layer> &b) {
                      return (std::get<0>(a) < std::get<0>(b));
                  });

        auto loc_source_location = source.getLocation();
        for (auto &inter: intersections) {
            auto loc_receiver_location = std::get<1>(inter);

            std::array<float, 3> vec{{loc_receiver_location[0] - loc_source_location[0],
                                             loc_receiver_location[1] - loc_source_location[1],
                                             loc_receiver_location[2] - loc_source_location[2]}};

            float norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

            vec[0] /= norm_vec;
            vec[1] /= norm_vec;
            vec[2] /= norm_vec;

            auto layer = getLocationLayer({loc_source_location[0] + vec[0],
                                           loc_source_location[1] + vec[1],
                                           loc_source_location[2] + vec[2]});

            segmentsP.emplace_back(loc_source_location, loc_receiver_location, layer, std::get<2>(inter).getTop());
            loc_source_location = loc_receiver_location;
        }

        auto layer = getLocationLayer(receiver_location);
        segmentsP.emplace_back(loc_source_location, receiver_location, layer, layer.getTop());
        segmentsS = segmentsP;
    }

    Layer Ray::getLocationLayer(std::array<float, 3> location) {

        std::vector<Layer> higher;
        std::copy_if(velocity_model.getLayers().begin(), velocity_model.getLayers().end(), std::back_inserter(higher),
                     [&location](Layer layer) {
                         auto loc = layer.getTop().getDepth({location[0], location[1]});
                         auto result = loc > location[2];
                         return result;
                     });
        auto it = std::min_element(higher.begin(), higher.end(),
                                   [&location](Layer &a, Layer &b) {
                                       return a.getTop().getDepth({location[0], location[1]}) - location[3] >
                                              a.getTop().getDepth({location[0], location[1]}) - location[3];
                                   });
        return *it;
    }

    std::vector<std::array<float, 3>> Ray::getTrajectoryP() {
        std::vector<std::array<float, 3>> trajectory;
        trajectory.push_back(source.getLocation());
        for (auto &seg: segmentsP) {
            trajectory.push_back(seg.getReceiver_location());
        }
        return trajectory;
    }

    std::vector<std::array<float, 3>> Ray::getTrajectoryS() {
        std::vector<std::array<float, 3>> trajectory;
        trajectory.push_back(source.getLocation());
        for (auto &seg: segmentsS) {
            trajectory.push_back(seg.getReceiver_location());
        }
        return trajectory;
    }

    void Ray::computePath() {
        computeSegments();
        optimizeTrajectory();
    }

    void Ray::optimizeTrajectory() {
        auto trajectory = getTrajectoryP();

        auto number_of_unknowns = (trajectory.size() - 2) * 2;
        vnl_vector<double> x(number_of_unknowns);

        for (unsigned long i = 0; i < number_of_unknowns / 2; i++) {
            x[2 * i] = trajectory[i + 1][0];
            x[2 * i + 1] = trajectory[i + 1][1];
        }

        cost_function function(this, static_cast<int>(number_of_unknowns), WaveType::WaveP);

        vnl_amoeba solver(function);
        solver.minimize(x);
        for (int i = 0; i < number_of_unknowns / 2; i++) {
            segmentsP[i].setReceiver_location({{static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
                                                      segmentsP[i].getHorizon().getDepth({static_cast<float>(x[2 * i]),
                                                                                         static_cast<float>(x[2 * i +
                                                                                                              1])})}});
        }

        timeP = static_cast<float>(function.f(x));

        trajectory = getTrajectoryS();
        for (unsigned long i = 0; i < number_of_unknowns / 2; i++) {
            x[2 * i] = trajectory[i + 1][0];
            x[2 * i + 1] = trajectory[i + 1][1];
        }
        cost_function functionS(this, static_cast<int>(number_of_unknowns), WaveType::WaveS);

        vnl_amoeba solverS(functionS);
        solverS.minimize(x);
        for (int i = 0; i < number_of_unknowns / 2; i++) {
            segmentsS[i].setReceiver_location({{static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
                                                       segmentsS[i].getHorizon().getDepth({static_cast<float>(x[2 * i]),
                                                                                           static_cast<float>(x[2 * i +
                                                                                                                1])})}});
        }

        timeS = static_cast<float>(functionS.f(x));
    }

    rapidjson::Document Ray::toJSON() {
        rapidjson::Document doc;
        rapidjson::Value json_val;
        rapidjson::Value tmp_json_val;
        doc.SetObject();

        auto &allocator = doc.GetAllocator();

        json_val.CopyFrom(source.toJSON(), allocator);
        doc.AddMember("Source", json_val, allocator);

        json_val.CopyFrom(receiver.toJSON(), allocator);
        doc.AddMember("Receiver", json_val, allocator);

        json_val.SetArray();
        auto trajectory = getTrajectoryP();
        for (const auto &tr_seg: trajectory) {
            tmp_json_val.SetArray();
            tmp_json_val.PushBack(tr_seg[0], allocator).
                    PushBack(tr_seg[1], allocator).
                    PushBack(tr_seg[2], allocator);
            json_val.PushBack(tmp_json_val, allocator);
        }
        doc.AddMember("TrajectoryP", json_val, allocator);

        json_val.SetArray();
        trajectory = getTrajectoryS();
        for (const auto &tr_seg: trajectory) {
            tmp_json_val.SetArray();
            tmp_json_val.PushBack(tr_seg[0], allocator).
                    PushBack(tr_seg[1], allocator).
                    PushBack(tr_seg[2], allocator);
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

        return doc;
    }

    bool Ray::checkRayCode(const std::vector<std::tuple<int, float, int>> &ray_code) const {
        /* check layers */
        for (std::tuple<int, float, int> ray : ray_code) {
            float ray_depth = std::get<1>(ray); // NOTE: or number
            bool correct = false; // if exist layer with number or exist layer on this depth then correct
            for (auto &layer : velocity_model.getLayers()) {
                float depth = layer.getTop().depth;
                if (ray_depth == depth) {
                    correct = true;
                    break;
                }
            }
            if (!correct) { // if it doesn't exist
                return false;
            }
        }

        /* check the source layer */
        const std::array<float, 3> source_layer_location = source.getLocation();
        float ray_code_depth = std::get<1>(ray_code[0]); // NOTE: or number
        if (source_layer_location[2] != ray_code_depth) { // if depth source (z value) != depth start ray_code
            return false;
        }

        /* check the receivers layer */
        const std::array<float, 3> receiver_layer_location = receiver.getLocation();
        ray_code_depth = std::get<1>(ray_code[ray_code.size()]); // NOTE: or number
        if (receiver_layer_location[2] != ray_code_depth) { // if depth receiver (z value) != depth end ray_code
            return false;
        }

        /* check the sequence of directions and layers
         * changing of directions is occurred inside the same layer only
         */

        // TODO: do correct or incorrect check
    }

    double Ray::cost_function::f(const vnl_vector<double> &x) {
        auto n = get_number_of_unknowns();
        std::vector<std::array<float, 3>> trajectory;
        std::array<float, 3> source_location;
        double time = 0;

        switch (type) {
            case WaveType::WaveP:
                trajectory = ray->getTrajectoryP();
                for (int i = 0; i < n / 2; i++) {
                    trajectory[i + 1] = {{static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
                                                 ray->segmentsP[i].getHorizon().getDepth(
                                                         {static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1])})}};
                }
                source_location = trajectory[0];
                for (auto &seg: ray->segmentsP) {
                    auto receiver_location = trajectory[&seg - &ray->segmentsP[0] + 1];

                    std::array<float, 3> vec{receiver_location[0] - source_location[0],
                                             receiver_location[1] - source_location[1],
                                             receiver_location[2] - source_location[2]};

                    float norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

                    auto layer = seg.getLayer();

                    time += norm_vec / layer.getVp();

                    source_location = receiver_location;
                }
                break;

            case WaveType::WaveS:
                trajectory = ray->getTrajectoryS();
                for (int i = 0; i < n / 2; i++) {
                    trajectory[i + 1] = {{static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
                                                 ray->segmentsS[i].getHorizon().getDepth(
                                                         {static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1])})}};
                }
                source_location = trajectory[0];
                for (auto &seg: ray->segmentsS) {
                    auto receiver_location = trajectory[&seg - &ray->segmentsS[0] + 1];

                    std::array<float, 3> vec{receiver_location[0] - source_location[0],
                                             receiver_location[1] - source_location[1],
                                             receiver_location[2] - source_location[2]};

                    float norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

                    auto layer = seg.getLayer();

                    time += norm_vec / layer.getVs();

                    source_location = receiver_location;
                }
        }

        return time;
    }

    Ray::cost_function::cost_function(Ray *ray, int number_of_unknowns, enum WaveType type) : ray(ray),
                                                                          vnl_cost_function(number_of_unknowns), type(type) {

    }
}