#ifndef LEASTSQUARESSOLVER_H
#define LEASTSQUARESSOLVER_H


#include <Eigen/Dense>
#include <ctime>
#include <hippo_chain/include/common/Debugger.h>


class LeastSquaresSolver
{
protected:
    Debugger debugger;

public:
    const int SIZE;

    LeastSquaresSolver(const int size)
    : debugger("", "lsq_solver")
    , SIZE(size)
    {}

    virtual void solve(const Eigen::MatrixXd& B,
                       const Eigen::VectorXd& eta,
                       Eigen::VectorXd& nu) = 0;
};


#endif  // LEASTSQUARESSOLVER_H