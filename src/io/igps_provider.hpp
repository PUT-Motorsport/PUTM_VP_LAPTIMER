#pragma once
#include "track/track_types.hpp"
#include <functional>

class IGpsProvider {
public:
    using GpsCallback = std::function<void(const GpsSample&)>;
    using SpeedCallback = std::function<void(double)>;

    virtual ~IGpsProvider() = default;
    virtual void setGpsCallback(GpsCallback cb) = 0;
    virtual void setSpeedCallback(SpeedCallback cb) = 0;
};
