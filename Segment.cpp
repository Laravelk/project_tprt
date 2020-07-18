#include "Segment.hpp"

namespace ray_tracing {
    Segment::Segment(const std::array<float, 3> &source_location, const std::array<float, 3> &receiver_location,
                     const Layer &layer, const FlatHorizon &horizon) : source_location(source_location),
                                                                       receiver_location(receiver_location),
                                                                       layer(layer),
                                                                       horizon(horizon) {}

    const std::array<float, 3> &Segment::getSource_location() const {
        return source_location;
    }

    const std::array<float, 3> &Segment::getReceiver_location() const {
        return receiver_location;
    }

    void Segment::setReceiver_location(const std::array<float, 3> &receiver_location) {
        Segment::receiver_location = receiver_location;
    }

    const FlatHorizon &Segment::getHorizon() const {
        return horizon;
    }

    const Layer &Segment::getLayer() const {
        return layer;
    }
}
