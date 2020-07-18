#include <utility>

#ifndef TPRT_VELOCITYMODEL_HPP
#define TPRT_VELOCITYMODEL_HPP


/*
 *
 *
 * */

#include <vector>
#include "Layer.hpp"
namespace ray_tracing {
    class VelocityModel {
        std::vector<Layer> layers;

    public:
        VelocityModel(std::vector<Layer> layers) : layers(std::move(layers)) {}

        const std::vector<Layer> &getLayers() const;

        rapidjson::Document toJSON();

        static VelocityModel fromJSON(const rapidjson::Value &doc);
    };
}

#endif //TPRT_VELOCITYMODEL_HPP
