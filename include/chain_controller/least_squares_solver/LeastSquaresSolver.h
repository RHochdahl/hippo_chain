#ifndef LEASTSQUARESSOLVER_H
#define LEASTSQUARESSOLVER_H


#include <Eigen/Dense>


class LeastSquaresSolver
{
public:
    const int SIZE;
protected:
    Eigen::VectorXd solution;
public:
    LeastSquaresSolver(const int size)
    : SIZE(size)
    , solution(Eigen::VectorXd::Zero(SIZE))
    {}

    virtual const Eigen::VectorXd& solve(const Eigen::MatrixXd& B, const Eigen::VectorXd& eta) = 0;
};


#endif  // LEASTSQUARESSOLVER_H