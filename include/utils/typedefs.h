#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include "defines.h"
#include <Eigen/Dense>

namespace Eigen {
    typedef Eigen::Matrix<double, 1, 1> Vector1d;
    typedef Eigen::Matrix<double, 3, 1> Vector3d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 7, 1> Vector7d;

    typedef Eigen::Matrix<double, 1, 1> Matrix1d;
    typedef Eigen::Matrix<double, 3, 3> Matrix3d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
};

#endif  // TYPEDEFS_H