#ifndef BOUNDSEIGEN_H
#define BOUNDSEIGEN_H


#include "Bounds.h"
#include <hippo_chain/include/common/sharedAlgorithms.hpp>


template<std::size_t SIZE>
class BoundsEigen : public Bounds<Eigen::Matrix<double, SIZE, 1>>
{
public:
    BoundsEigen(const std::vector<double>& poseUpper, const std::vector<double>& poseLower, const std::vector<double>& twist)
    : Bounds<Eigen::Matrix<double, SIZE, 1>>(poseUpper, poseLower, twist)
    {}

    bool checkBounds(const Eigen::Matrix<double, SIZE, 1>& actPose, const Eigen::Matrix<double, SIZE, 1>& actTwist) const
    {
        {
            auto poseIt = actPose.data();
            auto poseLLimIt = this->poseLowerLimit.data();
            auto poseULimIt = this->poseUpperLimit.data();
            const auto end = poseIt + SIZE;
            for (; poseIt!=end; poseIt++, poseLLimIt++, poseULimIt++) {
                if (*poseIt > *poseULimIt || *poseIt < *poseLLimIt) return false;
            }
        }

        {
            auto twistIt = actTwist.data();
            auto twistLimIt = this->twistLimit.data();
            const auto end = twistIt + SIZE;
            for (; twistIt!=end; twistIt++, twistLimIt++) {
                if (std::abs(*twistIt) > *twistLimIt) return false;
            }
        }

        return true;
    }

    bool enforceBounds(Eigen::Matrix<double, SIZE, 1>& desPose, Eigen::Matrix<double, SIZE, 1>& desTwist) const
    {
        bool inBounds = true;
        {
            auto poseIt = desPose.data();
            auto poseLLimIt = this->poseLowerLimit.data();
            auto poseULimIt = this->poseUpperLimit.data();
            const auto end = poseIt + SIZE;
            for (; poseIt!=end; poseIt++, poseLLimIt++, poseULimIt++) {
                if (*poseIt > *poseULimIt) {
                    *poseIt = *poseULimIt;
                    inBounds = false;
                    continue;
                }
                if (*poseIt < *poseLLimIt) {
                    *poseIt = *poseLLimIt;
                    inBounds = false;
                    continue;
                }
            }
        }

        if (!inBounds) {
            desTwist.fill(0);
            return false;
        }

        {
            auto twistIt = desTwist.data();
            auto twistLimIt = this->twistLimit.data();
            const auto end = twistIt + SIZE;
            for (; twistIt!=end; twistIt++, twistLimIt++) {
                if (std::abs(*twistIt) > *twistLimIt) {
                    *twistIt = shared::sgn(*twistIt) * (*twistLimIt);
                    inBounds = false;
                }
            }
        }

        return inBounds;
    }
};


#endif  // BOUNDSEIGEN_H