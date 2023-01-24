#ifndef MATLABLSQSOLVER_H
#define MATLABLSQSOLVER_H


#include <memory>
#include <algorithm>
#include <iostream>
#include "LeastSquaresSolver.h"

#include <cstddef>
#include <cstdlib>

// Matlab
#include <hippo_chain/lib/matlab_lsq_solver/LSQSolver.h>
#include <hippo_chain/lib/matlab_lsq_solver/rtwtypes.h>
#include <hippo_chain/lib/matlab_lsq_solver/rt_nonfinite.h>
#include <hippo_chain/lib/matlab_lsq_solver/coder_array.h>



class MatlabLSQSolver : public LeastSquaresSolver
{
private:
    LSQSolver solver;
    coder::array<double, 2U> A;
    coder::array<double, 1U> b;
    coder::array<double, 1U> x;
    coder::array<double, 1U> x0;


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
    , x()
    , x0()
    {
        x0.set_size(SIZE);
        for (auto it=x0.begin(), end=x0.end(); it!=end; it++) *it = 0;
    }


    void solve(const Eigen::MatrixXd& B,
               const Eigen::VectorXd& eta,
               Eigen::VectorXd& nu)
    {
        assert(B.rows() == eta.rows());
        assert(B.cols() == SIZE);
        assert(nu.rows() == SIZE);

#ifndef NDEBUG
        const clock_t begin_time = clock();

        debugger.addEntry("Size", SIZE);
        debugger.addEntry("B", B);
        debugger.addEntry("desired eta", eta);
#endif  // NDEBUG

        A.set_size(B.rows(), B.cols());
        std::copy(B.data(), B.data()+B.size(), A.data());

        b.set_size(eta.rows());
        std::copy(eta.data(), eta.data()+eta.size(), b.data());

        x.set(nu.data(), SIZE);

        solver.solveLSQ(A, b, x0, x);

        // std::copy(x.begin(), x.end(), x0.begin());

#ifndef NDEBUG
        double time = double(clock() - begin_time) * (1000.0/CLOCKS_PER_SEC);
        auto etaActual = B*nu;
        auto error = etaActual-eta;

        debugger.addEntry("nu", nu);
        debugger.addEntry("actual eta", etaActual);
        debugger.addEntry("error", error);
        debugger.addEntry("error norm", error.norm());
        debugger.addEntry("Solve time (in ms)", time);
        debugger.publish();
#endif  // NDEBUG
    }
};


#endif  // MATLABLSQSOLVER_H