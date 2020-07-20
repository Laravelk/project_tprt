#ifndef TPRT_SEGMENT_HPP
#define TPRT_SEGMENT_HPP


#include <array>
#include "Layer.hpp"

/*
 * часть луча от источника до границы, от границы до границы, от границы до приемника
 * */
namespace ray_tracing {
    class Segment {
        std::array<float, 3> source_location;
        std::array<float, 3> receiver_location;
        Layer layer;
        GridHorizon horizon;

    public:
        const Layer &getLayer() const;

        const GridHorizon &getHorizon() const;

        const std::array<float, 3> &getSource_location() const;

        const std::array<float, 3> &getReceiver_location() const;

    public:
        Segment(const std::array<float, 3> &source_location, const std::array<float, 3> &receiver_location,
                const Layer &layer, const GridHorizon &horizon);

        void setReceiver_location(const std::array<float, 3> &receiver_location);
    };
}

#endif //TPRT_SEGMENT_HPP
