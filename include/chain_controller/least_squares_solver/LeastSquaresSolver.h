#ifndef LEASTSQUARESSOLVER_H
#define LEASTSQUARESSOLVER_H


#include <Eigen/Dense>


class LeastSquaresSolver
{
public:
    const int SIZE;
public:
    LeastSquaresSolver(const int size)
    : SIZE(size)
    {}

    virtual void solve(const Eigen::Ref<const Eigen::MatrixXd>& B,
                       const Eigen::Ref<const Eigen::VectorXd>& eta,
                       Eigen::Ref<Eigen::VectorXd> nu) = 0;
};


#endif  // LEASTSQUARESSOLVER_H