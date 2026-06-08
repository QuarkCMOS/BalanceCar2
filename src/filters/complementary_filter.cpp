#include "complementary_filter.h"

ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha(clampAlpha(alpha))
    , angle(0.0f)
    , initialized(false)
{
}

void ComplementaryFilter::reset(float initialAngleDeg)
{
    angle = initialAngleDeg;

    initialized = true;
}

float ComplementaryFilter::update(
    float accelAngleDeg,
    float gyroRateDegPerS,
    float dt)
{
    if (dt <= 0.0f)
    {
        return angle;
    }

    // First update
    if (!initialized)
    {
        angle = accelAngleDeg;

        initialized = true;

        return angle;
    }

    // Gyro integration
    float gyroAngle =
        angle + gyroRateDegPerS * dt;

    // Complementary filter
    angle =
        alpha * gyroAngle +
        (1.0f - alpha) * accelAngleDeg;

    return angle;
}

float ComplementaryFilter::getAngle() const
{
    return angle;
}

float ComplementaryFilter::clampAlpha(float a)
{
    if (a < 0.0f)
    {
        return 0.0f;
    }

    if (a > 1.0f)
    {
        return 1.0f;
    }

    return a;
}