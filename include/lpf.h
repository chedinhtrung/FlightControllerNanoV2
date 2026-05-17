#ifndef LPF_H
#define LPF_H

#include <Arduino.h>
#include "datastructs.h"

// y[n] = y[n-1] + alpha * (x[n] - y[n-1])
class LPF
{
public:
    // alpha in [0, 1], where 0 = no update, 1 = no filtering.
    explicit LPF(float alpha = 1.0f)
        : alpha_(clamp01(alpha)), y_(0.0f), initialized_(false) {}

    // Construct from cutoff frequency and sample time.
    // alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
    LPF(float cutoff_hz, float dt_s)
        : alpha_(compute_alpha(cutoff_hz, dt_s)), y_(0.0f), initialized_(false) {}

    void set_alpha(float alpha) { alpha_ = clamp01(alpha); }

    void set_cutoff(float cutoff_hz, float dt_s) { alpha_ = compute_alpha(cutoff_hz, dt_s); }

    void reset(float value = 0.0f)
    {
        y_ = value;
        initialized_ = true;
    }

    float update(float x)
    {
        if (!initialized_)
        {
            y_ = x;
            initialized_ = true;
            return y_;
        }
        y_ += alpha_ * (x - y_);
        return y_;
    }

    float value() const { return y_; }
    float alpha() const { return alpha_; }
    bool initialized() const { return initialized_; }

private:
    static float clamp01(float v)
    {
        if (v < 0.0f) return 0.0f;
        if (v > 1.0f) return 1.0f;
        return v;
    }

    static float compute_alpha(float cutoff_hz, float dt_s)
    {
        if (cutoff_hz <= 0.0f || dt_s <= 0.0f)
        {
            return 1.0f;
        }
        
        const float rc = 1.0f / (TWO_PI * cutoff_hz);
        return clamp01(dt_s / (rc + dt_s));
    }

    float alpha_;
    float y_;
    bool initialized_;
};

class Vec3LPF
{
public:
    explicit Vec3LPF(float alpha = 1.0f)
        : x_(alpha), y_(alpha), z_(alpha) {}

    Vec3LPF(float cutoff_hz, float dt_s)
        : x_(cutoff_hz, dt_s), y_(cutoff_hz, dt_s), z_(cutoff_hz, dt_s) {}

    void set_alpha(float alpha)
    {
        x_.set_alpha(alpha);
        y_.set_alpha(alpha);
        z_.set_alpha(alpha);
    }

    void set_cutoff(float cutoff_hz, float dt_s)
    {
        x_.set_cutoff(cutoff_hz, dt_s);
        y_.set_cutoff(cutoff_hz, dt_s);
        z_.set_cutoff(cutoff_hz, dt_s);
    }

    void reset(const Vec3 &value = Vec3{})
    {
        x_.reset(value.x);
        y_.reset(value.y);
        z_.reset(value.z);
    }

    Vec3 update(const Vec3 &input)
    {
        return Vec3{
            x_.update(input.x),
            y_.update(input.y),
            z_.update(input.z)
        };
    }

    Vec3 value() const
    {
        return Vec3{x_.value(), y_.value(), z_.value()};
    }

    bool initialized() const
    {
        return x_.initialized() && y_.initialized() && z_.initialized();
    }

private:
    LPF x_;
    LPF y_;
    LPF z_;
};

#endif
