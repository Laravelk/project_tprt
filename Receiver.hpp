#include <utility>

#ifndef TPRT_RECIEVER_HPP
#define TPRT_RECIEVER_HPP

#include <array>
#include <vector>
#include <string>
#include "rapidjson/document.h"

namespace ray_tracing {
    class Receiver {
        std::array<float, 3> location;
        std::vector<std::array<float, 3>> orientation;

        float sampling;

        std::string name;

    public:
        Receiver(std::array<float, 3> location, std::vector<std::array<float, 3>> orientation, float sampling,
                 std::string name = "") :
                location(location),
                orientation(std::move(orientation)),
                sampling(sampling),
                name(name) {
        }

        const std::array<float, 3> & getLocation() const {
            return location;
        }

        rapidjson::Document toJSON();

        static Receiver fromJSON(const rapidjson::Value &doc);
    };
}

#endif //TPRT_RECIEVER_HPP
