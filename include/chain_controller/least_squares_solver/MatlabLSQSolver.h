#ifndef MATLABLSQSOLVER_H
#define MATLABLSQSOLVER_H


#include <memory>
#include <algorithm>
#include <iostream>
#include "LeastSquaresSolver.h"

// Matlab
#include "../../../lib/matlab_lsq_solver/LSQSolver.h"
#include "../../../lib/matlab_lsq_solver/rtwtypes.h"
// #include <hippo_chain/lib/rtwtypes.h>
#include <cstddef>
#include <cstdlib>
#include "../../../lib/matlab_lsq_solver/LSQSolver.h"
#include "../../../lib/matlab_lsq_solver/rt_nonfinite.h"
#include "../../../lib/matlab_lsq_solver/coder_array.h"



class MatlabLSQSolver : public LeastSquaresSolver
{
private:
    LSQSolver solver;
    coder::array<double, 2U> A;
    coder::array<double, 1U> b;
    coder::array<double, 1U> x;


    static inline coder::array<double, 1U> EigenVec2MatlabArray(Eigen::VectorXd& vec)
    {
        coder::array<double, 1U> result;
        result.set(vec.data(), static_cast<int>(vec.rows()));
        return result;
    }

    static inline coder::array<double, 2U> EigenMat2MatlabArray(Eigen::MatrixXd& mat)
    {
        coder::array<double, 2U> result;
        result.set(mat.data(), static_cast<int>(mat.rows()), static_cast<int>(mat.cols()));
        return result;
    }


public:
    MatlabLSQSolver(const int size)
    : LeastSquaresSolver(size)
    , solver()
    , A()
    , b()
    , x(EigenVec2MatlabArray(solution))
    {}


    const Eigen::VectorXd& solve(const Eigen::MatrixXd& B, const Eigen::VectorXd& eta)
    {
        assert(B.rows() == eta.rows());
        assert(B.cols() == SIZE);

        if (eta.norm() < 1e-12) {
            solution.fill(0);
            return solution;
        }

        A.set_size(B.rows(), B.cols());
        std::copy(B.data(), B.data()+B.size(), A.data());

        b.set_size(eta.rows());
        std::copy(eta.data(), eta.data()+eta.size(), b.data());

        solver.solve(A, b, x);

        return solution;
    }
};


#endif  // MATLABLSQSOLVER_H