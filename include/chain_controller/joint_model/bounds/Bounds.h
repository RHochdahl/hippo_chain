#ifndef BOUNDS_H
#define BOUNDS_H


#include <Eigen/Dense>
#include <vector>


template<typename StateVector>
class Bounds
{
protected:
    const StateVector poseUpperLimit;
    const StateVector poseLowerLimit;
    const StateVector twistLimit;

private:
    StateVector toStateVector(const std::vector<double>& vec)
    {
        if constexpr (std::is_same<StateVector, double>::value) {
            assert(vec.size() == 1);
            return vec.front();
        } else {
            assert(vec.size() == StateVector::RowsAtCompileTime);
            return StateVector(vec.data());
        }
    }


public:
    Bounds(const std::vector<double>& poseUpper, const std::vector<double>& poseLower, const std::vector<double>& twist)
    : poseUpperLimit(toStateVector(poseUpper))
    , poseLowerLimit(toStateVector(poseLower))
    , twistLimit(toStateVector(twist))
    {
        if constexpr (std::is_same<StateVector, double>::value) {
            assert(poseUpperLimit > poseLowerLimit);
        } else {
            const auto end = poseLowerLimit.data()+poseLowerLimit.size();
            for (auto lIt=poseLowerLimit.data(), uIt=poseUpperLimit.data(); lIt!=end; lIt++, uIt++) {
                assert(*uIt > *lIt);
            }
        }
    }

    virtual bool checkBounds(const StateVector& actPose, const StateVector& actTwist) const = 0;
    virtual bool enforceBounds(StateVector& desPose, StateVector& desTwist) const = 0;
};


#endif  // BOUNDS_H