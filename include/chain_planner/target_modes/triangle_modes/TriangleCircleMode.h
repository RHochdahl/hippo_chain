#ifndef TRIANGLECIRCLETARGETMODE_H
#define TRIANGLECIRCLETARGETMODE_H


#include "../ContinuousMode.h"

class TriangleCircleMode : public ContinuousMode
{
private:
    double centerX, centerY, radius, depth, frequency;
    boost::function<void(double, double, double, double, double, double, double, double)> setBaseFunction;

    void step(const double time)
    {
        const double phase = frequency*time;
        const double sinPhase = std::sin(phase);
        const double cosPhase = std::cos(phase);
        const double x = centerX + radius*std::sin(phase);
        const double y = centerY - radius*std::cos(phase);

        const double vMax = frequency*radius;
        const double u = vMax*std::cos(phase);
        const double v = vMax*std::sin(phase);

        setBaseFunction(x, y, depth, phase, u, v, 0.0, frequency);
    }


public:
    TriangleCircleMode(const double _centerX,
                       const double _centerY,
                       const double _radius,
                       const double _depth,
                       const double _period,
                       const double _duration,
                       boost::function<void(double, double, double, double, double, double, double, double)> _setBaseFunction)
    : ContinuousMode(_duration)
    , centerX(_centerX)
    , centerY(_centerY)
    , radius(_radius)
    , depth(_depth)
    , frequency(2*M_PI/_period)
    , setBaseFunction(_setBaseFunction)
    {}

    ~TriangleCircleMode() = default;
};

#endif  // TRIANGLECIRCLETARGETMODE_H