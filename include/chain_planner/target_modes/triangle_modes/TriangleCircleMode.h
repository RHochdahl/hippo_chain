#ifndef TRIANGLECIRCLETARGETMODE_H
#define TRIANGLECIRCLETARGETMODE_H


#include "../ContinuousMode.h"

class TriangleCircleMode : public ContinuousMode
{
private:
    double centerX, centerY, radius, depth, frequency;
    boost::function<void(Target4d, Target4d, Target4d)> setBaseFunction;

    void step(const double time)
    {
        const double phase = frequency*time;

        const Target4d poseTarget{centerX + radius*std::sin(phase), centerY - radius*std::cos(phase), depth, phase};
        const Target4d twistTarget{frequency*radius, 0.0, 0.0, frequency};
        const Target4d accelTarget{0.0, frequency*twistTarget.x, 0.0, 0.0};

        setBaseFunction(poseTarget, twistTarget, accelTarget);
    }


public:
    TriangleCircleMode(const double _centerX,
                       const double _centerY,
                       const double _radius,
                       const double _depth,
                       const double _period,
                       const double _duration,
                       boost::function<void(Target4d, Target4d, Target4d)> _setBaseFunction)
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