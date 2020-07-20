#ifndef TPRT_LAYER_HPP
#define TPRT_LAYER_HPP


#include <string>
#include "rapidjson/document.h"
#include "Horizon/FlatHorizon.hpp"
#include "Horizon/GridHorizon.h"


/*
 * Среда
 * */
namespace ray_tracing {
    class Layer {

    public:
        float Vp;
        float Vs;

        GridHorizon top;
        std::string name;

        Layer(float Vp, float Vs, GridHorizon &top, std::string name = "") : Vp(Vp), Vs(Vs), top(top), name(name) {}

        Layer(const Layer &rhs) : Vp(rhs.Vp), Vs(rhs.Vs), top(rhs.top), name(rhs.name) {}
        Layer &operator=(const Layer &rhs) {
            Vp = rhs.Vp;
            Vs = rhs.Vs;
            top = rhs.top;
            name = rhs.name;
            return *this;
        }

        float getVp() const {
            return Vp;
        }

        float getVs() const {
            return Vs;
        }

        const GridHorizon &getTop() const {
            return top;
        }

        rapidjson::Document toJSON();

        static Layer fromJSON(const rapidjson::Value &doc);
    };

}
#endif //TPRT_LAYER_HPP
