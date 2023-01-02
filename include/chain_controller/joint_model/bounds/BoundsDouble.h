#ifndef BOUNDSDOUBLE_H
#define BOUNDSDOUBLE_H


#include "Bounds.h"
#include "../../../utils/sharedAlgorithms.hpp"


class BoundsDouble : public Bounds<double>
{
public:
    BoundsDouble(const double poseUpper, const double poseLower, const double twist)
    : Bounds(std::vector<double>(1, poseUpper), std::vector<double>(1, poseLower), std::vector<double>(1, twist))
    {}

    bool checkBounds(const double& actPose, const double& actTwist) const
    {
        if (actPose > poseUpperLimit || actPose < poseLowerLimit) return false;
        if (std::abs(actTwist) > twistLimit) return false;
        return true;
    }

    bool enforceBounds(double& desPose, double& desTwist) const
    {
        bool inBounds = true;
        if (desPose > poseUpperLimit) {
            desPose = poseUpperLimit;
            inBounds = false;
        } else if (desPose < poseLowerLimit) {
            desPose = poseLowerLimit;
            inBounds = false;
        }

        if (!inBounds) {
            desTwist = 0;
            return false;
        }

        if (std::abs(desTwist) > twistLimit) {
            desTwist = shared::sgn(desTwist) * twistLimit;
            return false;
        }
    
        return inBounds;
    }
};


#endif  // BOUNDSDOUBLE_H