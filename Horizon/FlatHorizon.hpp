#include <utility>

#ifndef TPRT_FLATHORIZON_HPP
#define TPRT_FLATHORIZON_HPP


#include <cmath>
#include <array>
#include <iostream>
#include "../rapidjson/document.h"
#include "Horizon.h"

namespace ray_tracing {
    class FlatHorizon : public Horizon {

    public:
        float dip;
        float azimuth;
        float depth;

        std::array<float, 2> anchor;
        std::array<float, 3> normal;
        std::string name;


        FlatHorizon(float depth, float dip, float azimuth, std::array<float, 2> anchor = {{0, 0}},
                    std::string name = "");

        virtual float getDepth(std::array<float, 2> x) const override;

        virtual std::array<float, 3> calcIntersect(const std::array<float, 3> &x0, const std::array<float, 3> &x1)
                                            const override;

        virtual rapidjson::Document toJSON() override;

        static FlatHorizon fromJSON(const rapidjson::Value &doc);
    };

}

#endif //TPRT_FLATHORIZON_HPP
