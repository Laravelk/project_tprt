#ifndef TPRT_HORIZON_H
#define TPRT_HORIZON_H

//
// Created by Иван Морозов on 2020-06-17.
//

#include "../rapidjson/pointer.h"

namespace ray_tracing {
    class Horizon {
    public:
        virtual rapidjson::Document toJSON() = 0;

        virtual float getDepth(std::array<float, 2> x) const = 0;

        virtual std::array<float, 3> calcIntersect(const std::array<float, 3> &x0,
                                                   const std::array<float, 3> &x1) const = 0;

    private:
    };
}


#endif //TPRT_HORIZON_H
