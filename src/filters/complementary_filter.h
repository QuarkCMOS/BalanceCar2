#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

class ComplementaryFilter
{
public:
    explicit ComplementaryFilter(float alpha = 0.98f);

    void reset(float initialAngleDeg = 0.0f);

    float update(float accelAngleDeg,
                 float gyroRateDegPerS,
                 float dt);

    float getAngle() const;

private:
    float clampAlpha(float a);

private:
    float alpha;

    float angle;

    bool initialized;
};

#endif