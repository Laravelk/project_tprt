#ifndef TPRT_SOURCE_HPP
#define TPRT_SOURCE_HPP

#include <array>
#include <string>
#include "rapidjson/document.h"

namespace ray_tracing {
    class Source {
        std::array<float, 3> location;
        std::array<std::array<float, 3>, 3> moment;
        float magnitude;
        float t0;

        std::string type;

    public:
        Source(std::array<float, 3> location, std::array<std::array<float, 3>, 3> moment, float magnitude, float t0,
               std::string type = "") :
                location(location),
                moment(moment),
                magnitude(magnitude),
                t0(t0),
                type(type) {}

        const std::array<float, 3> & getLocation() const;

        rapidjson::Document toJSON();

        static Source fromJSON(const rapidjson::Value &doc);
    };
}

#endif //TPRT_SOURCE_HPP
